#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <numbers>
#include <tuple>

#include "globals.hpp"
#include "terrain.hpp"
#include "triple_buffer.hpp"
#include "vec.hpp"

namespace td {

    static_assert(std::atomic_uint8_t::is_always_lock_free);

    template<typename T>
        requires (std::floating_point<T>)
    class Oscillator {
    private:
        static constexpr size_t kTextureResolution = (1ull << 12);

    public:
        explicit Oscillator(std::filesystem::path const &terrain_file) : terrain_{ terrain_file } {
            InitVoices();
            InitPortals();
        }

        void GetNextBlock(std::array<std::array<float, kBlockSize>, 2> &block) {
            CollectVoiceChanges();
            ApplyVoiceDeactivations();
            ApplyActiveVoiceChanges();

            for (size_t i = 0; i < 2; ++i) {
                block[i].fill(0.0f);
            }

            std::array<std::array<float, kMaxVoices>, kBlockSize> voices_block{};
            for (size_t i = 0; i < kBlockSize; ++i) {
                voices_block[i] = GetVoicesSamples();
                AdvanceVoices();
            }

            for (size_t i = 0; i < 2; ++i) {
                for (size_t j = 0; j < kBlockSize; ++j) {
                    for (size_t k = 0; k < active_voices_; ++k) {
                        block[i][j] += voices_block[j][k] * voice_amp_[i][k];
                    }
                }
            }
        }

    private:
        // ############################## Voices ##############################

        using v_id_t = uint8_t;
        using p_id_t = uint8_t;
        using c_id_t = uint8_t;
        using v_gen_t = uint64_t;

        static v_id_t constexpr kMaxVoices = 32;
        static v_id_t constexpr kInvalidVoice = std::numeric_limits<v_id_t>::max();

        struct VoiceHandle {
            v_gen_t gen;
            v_id_t v_id;
        };

        struct VoiceParamPack {
            Vec3<T> pos;
            Vec2<T> dir;
            T freq;
            float gain;
            float pan;
        };

        struct VoiceParamGenerations {
            v_gen_t pos{};
            v_gen_t dir{};
            v_gen_t freq{};
            v_gen_t gain{};
            v_gen_t pan{};
        };

        struct VoiceState {
            v_gen_t gen{};
            VoiceParamGenerations param_gens;
        };

        struct VoicePendingState {
            v_gen_t gen{};
            VoiceParamPack params;
            VoiceParamGenerations param_gens;
            bool active{};
        };

    public:
        [[nodiscard]] std::optional<VoiceHandle> ActivateVoice(VoiceParamPack params) noexcept {
            auto &[pos, dir, freq, gain, pan] = params;
            if (!pos.IsUnit() ||
                !dir.IsUnit() ||
                freq < T{ 0.0 } || freq >= T{ kSampleRate / 2.0 } || !std::isfinite(freq) ||
                gain < T{ 0.0 } || !std::isfinite(gain) ||
                pan < T{ 0.0 } || pan > T{ 1.0 } || !std::isfinite(pan)) {
                return std::nullopt;
            }

            pos = pos.Norm();
            dir = dir.Norm();

            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                auto &pending_state = voice_pending_states_control_[v_id];
                if (pending_state.active) {
                    continue;
                }

                IncGen(pending_state.gen);
                pending_state.active = true;
                pending_state.params = params;
                IncParamGens(pending_state.param_gens);

                voice_state_buffers_[v_id].Publish(pending_state);

                return { VoiceHandle{ .gen = pending_state.gen, .v_id = v_id } };
            } 
            return std::nullopt;
        }

        [[nodiscard]] bool DeactivateVoice(VoiceHandle const &handle) noexcept {
            if (!IsVoiceActive(handle)) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.active = false;

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

        [[nodiscard]] bool SetVoicePos(VoiceHandle const &handle, Vec3<T> const &pos) noexcept {
            if (!IsVoiceActive(handle) || !pos.IsUnit()) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.params.pos = pos;
            IncGen(current_pending.param_gens.pos);

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

        [[nodiscard]] bool SetVoiceDir(VoiceHandle const &handle, Vec2<T> const &dir) noexcept {
            if (!IsVoiceActive(handle) || !dir.IsUnit()) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.params.dir = dir;
            IncGen(current_pending.param_gens.dir);

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

        [[nodiscard]] bool SetVoiceFreq(VoiceHandle const &handle, T const &freq) noexcept {
            if (!IsVoiceActive(handle) || 
                freq < T{ 0.0 } || freq >= T{ kSampleRate / 2.0 } || !std::isfinite(freq)) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.params.freq = freq;
            IncGen(current_pending.param_gens.freq);

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

        [[nodiscard]] bool SetVoiceGain(VoiceHandle const &handle, float const &gain) noexcept {
            if (!IsVoiceActive(handle) || 
                gain < 0.0f || !std::isfinite(gain)) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.params.gain = gain;
            IncGen(current_pending.param_gens.gain);

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

        [[nodiscard]] bool SetVoicePan(VoiceHandle const &handle, float const &pan) noexcept {
            if (!IsVoiceActive(handle) || 
                pan < T{ 0.0 } || pan > 1.0f || !std::isfinite(pan)) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.params.pan = pan;
            IncGen(current_pending.param_gens.pan);

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

    private:
        void SetVoicePosInternal(v_id_t const lane, Vec3<T> const &new_pos) {
            assert(lane < kMaxVoices);
            assert(new_pos.IsUnit());

            Vec3<T> &current_pos = voice_pos_[lane];
            Vec3<T> &current_dir = voice_dir_[lane];

            auto const [old_u, old_v] = CalculateVoiceUvs(current_pos);
            T const u_heading = current_dir * old_u;
            T const v_heading = current_dir * old_v;

            current_pos = new_pos.Norm();

            auto const [new_u, new_v] = CalculateVoiceUvs(new_pos);

            current_dir = (u_heading * new_u + v_heading * new_v).Norm();

            RecalculateVoiceCacheForVoice(lane);
        }

        void SetVoiceDirInternal(v_id_t const lane, Vec2<T> const &new_dir) {
            assert(lane < kMaxVoices);
            assert(new_dir.IsUnit());

            auto &pos = voice_pos_[lane];
            auto &dir = voice_dir_[lane];

            assert(pos.IsUnit());

            auto const [u, v] = CalculateVoiceUvs(pos);
            dir = new_dir.X() * u + new_dir.Y() * v;

            // sanitize orthonormal
            dir = (dir - (dir * pos) * pos).Norm();

            RecalculateVoiceCacheForVoice(lane);
        }

        void SetVoiceFreqInternal(v_id_t const lane, T const freq) {
            assert(lane < kMaxVoices);
            assert(freq >= 0.0f && freq < kSampleRate / 2 && std::isfinite(freq));

            voice_freq_[lane] = freq;
            voice_angular_inc_[lane] = T{ 2.0 } * std::numbers::pi_v<T> *freq / kSampleRate;
            voice_step_cos_[lane] = std::cos(voice_angular_inc_[lane]);
            voice_step_sin_[lane] = std::sin(voice_angular_inc_[lane]);
        }

        void SetVoiceGainInternal(v_id_t const lane, float const gain) {
            assert(lane < kMaxVoices);
            assert(gain >= 0.0f && std::isfinite(gain));

            voice_gain_[lane] = gain;
            CalculateAmp(lane);
        }

        void SetVoicePanInternal(v_id_t const lane, float const pan) {
            assert(lane < kMaxVoices);
            assert(pan >= 0.0f && pan <= 1.0f && std::isfinite(pan));

            voice_pan_[lane] = pan;
            CalculateAmp(lane);
        }

        void CalculateAmp(v_id_t const lane) const {
            voice_amp_[0][lane] = voice_gain_[lane] * std::cos(0.5f * std::numbers::pi_v<float> * voice_pan_[lane]);
            voice_amp_[1][lane] = voice_gain_[lane] * std::sin(0.5f * std::numbers::pi_v<float> * voice_pan_[lane]);
        }

        struct Uv { Vec3<T> u, v; };

        static Uv CalculateVoiceUvs(Vec3<T> const &pos) {
            Vec3<T> u = Vec3<T>{0.0, 0.0, 1.0}.Cross(pos);

            if (u.MagSq() <= kParallelEps<T>) {
                static Vec3<T> constexpr kUFallback = {1.0, 0.0, 0.0};
                u = (kUFallback - (kUFallback * pos) * pos).Norm();
            } else {
                u = u.Norm();
            }

            Vec3<T> const v = pos.Cross(u).Norm();

            return { .u = u, .v = v };
        }

        static void IncParamGens(VoiceParamGenerations &param_gens) noexcept {
            IncGen(param_gens.pos);
            IncGen(param_gens.dir);
            IncGen(param_gens.freq);
            IncGen(param_gens.gain);
            IncGen(param_gens.pan);
        }

        static void IncGen(v_gen_t &gen) noexcept {
            if (++gen == 0) { gen = 1; }
        }

        [[nodiscard]] bool IsVoiceActive(VoiceHandle const &handle) {
            if (handle.v_id >= kMaxVoices || handle.gen == 0) {
                return false;
            }
            
            auto &pending_state = voice_pending_states_control_[handle.v_id];
            return pending_state.active && handle.gen == pending_state.gen;
        }

        void InitVoices() {
            for (v_id_t id = 0; id < kMaxVoices; ++id) {
                voice_pos_[id] = {1.0, 0.0, 0.0};
                voice_dir_[id] = {0.0, 0.0, 1.0};
                SetVoiceFreqInternal(id, T{ 440.0 });
                SetVoicePanInternal(id, 0.5f);
                
                voice_next_circ_[id] = kInvalidCircle;
                voice_dist_to_circ_[id] = std::numeric_limits<T>::infinity();
            }

            slot_to_lane_.fill(kInvalidVoice);
            lane_to_slot_.fill(kInvalidVoice);
        }

        void CollectVoiceChanges() noexcept {
            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                if (auto pending = voice_state_buffers_[v_id].Consume()) {
                    voice_pending_states_audio_[v_id] = *pending;
                    voice_pending_flags_[v_id] = true;
                }
            }
        }

        void ApplyVoiceDeactivations() noexcept {
            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                if (!voice_pending_flags_[v_id] || voice_pending_states_audio_[v_id].active) {
                    continue;
                }

                if (v_id_t const lane = slot_to_lane_[v_id]; lane != kInvalidVoice) {
                    v_id_t const last_lane = --active_voices_;
                    if (lane != last_lane) {
                        std::swap(voice_pos_[lane], voice_pos_[last_lane]);
                        std::swap(voice_dir_[lane], voice_dir_[last_lane]);
                        std::swap(voice_freq_[lane], voice_freq_[last_lane]);
                        std::swap(voice_gain_[lane], voice_gain_[last_lane]);
                        std::swap(voice_pan_[lane], voice_pan_[last_lane]);
                        std::swap(voice_angular_inc_[lane], voice_angular_inc_[last_lane]);
                        std::swap(voice_next_circ_[lane], voice_next_circ_[last_lane]);
                        std::swap(voice_dist_to_circ_[lane], voice_dist_to_circ_[last_lane]);
                        std::swap(voice_step_sin_[lane], voice_step_sin_[last_lane]);
                        std::swap(voice_step_cos_[lane], voice_step_cos_[last_lane]);
                        std::swap(voice_amp_[0][lane], voice_amp_[0][last_lane]);
                        std::swap(voice_amp_[1][lane], voice_amp_[1][last_lane]);

                        v_id_t const moved_v_id = lane_to_slot_[last_lane];
                        slot_to_lane_[moved_v_id] = lane;
                        lane_to_slot_[lane] = moved_v_id;
                    }

                    lane_to_slot_[last_lane] = kInvalidVoice;
                    slot_to_lane_[v_id] = kInvalidVoice;
                }

                auto &current_state = voice_state_[v_id];
                auto const &new_state = voice_pending_states_audio_[v_id];
                current_state.gen = new_state.gen;
                current_state.param_gens = new_state.param_gens;

                voice_pending_flags_[v_id] = false;
            }
        }

        void ApplyActiveVoiceChanges() {
            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                if (!voice_pending_flags_[v_id]) {
                    continue;
                }

                auto const &pending = voice_pending_states_audio_[v_id];
                auto &current = voice_state_[v_id];

                if (v_id_t &lane = slot_to_lane_[v_id]; lane == kInvalidVoice || pending.gen != current.gen) {
                    if (lane == kInvalidVoice) {
                        assert(active_voices_ < kMaxVoices);

                        lane = active_voices_++;
                        lane_to_slot_[lane] = v_id;
                    }

                    SetVoicePosInternal(lane, pending.params.pos);
                    SetVoiceDirInternal(lane, pending.params.dir);
                    SetVoiceFreqInternal(lane, pending.params.freq);
                    SetVoiceGainInternal(lane, pending.params.gain);
                    SetVoicePanInternal(lane, pending.params.pan);

                    current.gen = pending.gen;
                } else {
                    if (!pending.active) {
                        continue;
                    }

                    if (pending.param_gens.pos != current.param_gens.pos) {
                        SetVoicePosInternal(lane, pending.params.pos);
                    }

                    if (pending.param_gens.dir != current.param_gens.dir) {
                        SetVoiceDirInternal(lane, pending.params.dir);
                    }

                    if (pending.param_gens.freq != current.param_gens.freq) {
                        SetVoiceFreqInternal(lane, pending.params.freq);
                    }

                    if (pending.param_gens.gain != current.param_gens.gain) {
                        SetVoiceGainInternal(lane, pending.params.gain);
                    }

                    if (pending.param_gens.pan != current.param_gens.pan) {
                        SetVoicePanInternal(lane, pending.params.pan);
                    }
                }
                current.param_gens = pending.param_gens;
                voice_pending_flags_[v_id] = false;
            }
        }

        [[nodiscard]] std::array<float, kMaxVoices> GetVoicesSamples() const {
            std::array<float, kMaxVoices> samples{};
            for (size_t i = 0; i < active_voices_; ++i) {
                samples[i] = terrain_.ReadPos(voice_pos_[i]);
            }
            return samples;
        }

        void AdvanceVoice(v_id_t const v_id) {
            assert(v_id < kMaxVoices);

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            auto new_pos = voice_step_cos_[v_id] * pos + voice_step_sin_[v_id] * dir;
            dir = voice_step_cos_[v_id] * dir - voice_step_sin_[v_id] * pos;
            pos = new_pos;
        } 

        void AdvanceVoice(v_id_t const v_id, T const angular_inc) {
            assert(v_id < kMaxVoices);

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            T const c = std::cos(angular_inc);
            T const s = std::sin(angular_inc);

            auto new_pos = c * pos + s * dir;
            dir = c * dir - s * pos;
            pos = new_pos;
        }

        void AdvanceVoices() {
            for (v_id_t v_id = 0; v_id < active_voices_; ++v_id) {
                T angular_inc = voice_angular_inc_[v_id];
                c_id_t next_circ = voice_next_circ_[v_id];
                T dist_to_circ = voice_dist_to_circ_[v_id];

                if (dist_to_circ > angular_inc) {
                    AdvanceVoice(v_id);
                    dist_to_circ -= angular_inc;
                    angular_inc = T{ 0.0 };
                } else {
                    HandlePortalCrossing(v_id, angular_inc, next_circ, dist_to_circ);
                }

                voice_next_circ_[v_id] = next_circ;
                voice_dist_to_circ_[v_id] = dist_to_circ;
            }
        }

        void HandlePortalCrossing(v_id_t const v_id, T angular_inc, c_id_t &next_circ, T &dist_to_circ) {
            while (angular_inc > T{ 0.0 }) {
                if (dist_to_circ > angular_inc) {
                    AdvanceVoice(v_id, angular_inc);
                    dist_to_circ -= angular_inc;
                    angular_inc = T{ 0.0 };
                } else {
                    AdvanceVoice(v_id, dist_to_circ);
                    angular_inc -= dist_to_circ;

                    Teleport(v_id, next_circ);

                    std::tie(next_circ, dist_to_circ) = FindNextIntersection(v_id);
                }
            }
        }
        
        void RecalculateVoiceCacheForVoice(v_id_t const v_id) const {
            std::tie(voice_next_circ_[v_id], voice_dist_to_circ_[v_id]) = FindNextIntersection(v_id);
        }

        void RecalculateVoiceCacheForNewCircle(c_id_t const c_id) const {
            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                T const dist = Intersect(v_id, c_id);

                if (dist < voice_dist_to_circ_[v_id]) {
                    voice_next_circ_[v_id] = c_id;
                    voice_dist_to_circ_[v_id] = dist;
                }
            }
        }

        void RecalculateVoiceCacheForRemovedCircle(c_id_t const c_id) const {
            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                if (voice_next_circ_[v_id] == c_id) {
                    std::tie(voice_next_circ_[v_id], voice_dist_to_circ_[v_id]) = FindNextIntersection(v_id);
                }
            }
        }

        // State
        std::array<Vec3<T>, kMaxVoices> voice_pos_{};
        std::array<Vec3<T>, kMaxVoices> voice_dir_{};
        std::array<T, kMaxVoices> voice_freq_{};
        std::array<float, kMaxVoices> voice_gain_{};
        std::array<float, kMaxVoices> voice_pan_{};
        v_id_t active_voices_ = 0;

        // Threading
        std::array<VoicePendingState, kMaxVoices> voice_pending_states_control_{};
        std::array<TripleBuffer<VoicePendingState>, kMaxVoices> voice_state_buffers_{};
        std::array<VoicePendingState, kMaxVoices> voice_pending_states_audio_{};
        std::bitset<kMaxVoices> voice_pending_flags_{};
        std::array<VoiceState, kMaxVoices> voice_state_{};
        std::array<v_id_t, kMaxVoices> slot_to_lane_{};
        std::array<v_id_t, kMaxVoices> lane_to_slot_{};

        // Caching
        mutable std::array<T, kMaxVoices> voice_angular_inc_{};
        mutable std::array<c_id_t, kMaxVoices> voice_next_circ_{};
        mutable std::array<T, kMaxVoices> voice_dist_to_circ_{};
        mutable std::array<T, kMaxVoices> voice_step_sin_{};
        mutable std::array<T, kMaxVoices> voice_step_cos_{};
        mutable std::array<std::array<float, kMaxVoices>, 2> voice_amp_{};
        
        // ############################## Portals #############################

        static p_id_t constexpr kMaxPortals = 1 << 6; // this keeps c_ids from loosing portal id_bits
        static c_id_t constexpr kMaxCircles = c_id_t{ 2 } * kMaxPortals;
        static c_id_t constexpr kInvalidCircle = std::numeric_limits<c_id_t>::max();

    public:
        bool IsPortalActive(p_id_t const p_id) const {
            assert(p_id < kMaxPortals);

            for (size_t i = 0; i < active_portals_; ++i) {
                if (portals_[i] == p_id) { return true; }
            }

            return false;
        }

        bool ActivatePortalInternal(p_id_t const p_id) {
            if (p_id >= kMaxPortals || active_portals_ >= kMaxPortals || IsPortalActive(p_id)) {
                return false;
            }

            portals_[active_portals_++] = p_id;
            RecalculateVoiceCacheForNewCircle(p_id << 1 | 0);
            RecalculateVoiceCacheForNewCircle(p_id << 1 | 1);
            return true;
        }

        bool DeactivatePortalInternal(p_id_t const p_id) {
            if (p_id >= kMaxPortals) {
                return false;
            }

            for (size_t i = 0; i < active_portals_; ++i) {
                if (portals_[i] == p_id) {
                    std::swap(portals_[i], portals_[--active_portals_]);

                    RecalculateVoiceCacheForRemovedCircle(p_id << 1 | 0);
                    RecalculateVoiceCacheForRemovedCircle(p_id << 1 | 1);

                    return true;
                }
            }

            return false;
        }

        void SetPortalRotationInternal(p_id_t const p_id, T const rot) {
            assert(p_id < kMaxPortals);

            portal_rot_cos_[p_id] = std::cos(rot);
            portal_rot_sin_[p_id] = std::sin(rot);
        }

        void SetCircleAxisInternal(c_id_t const c_id, Vec3<T> axis) {
            assert(c_id < kMaxCircles);
            assert(axis.IsUnit());

            circle_axis_[c_id] = axis.Norm();
            CalculateCircleUvs(c_id >> 1);
            
            if (IsPortalActive(c_id >> 1)) {
                RecalculateVoiceCacheForRemovedCircle(c_id);
                RecalculateVoiceCacheForNewCircle(c_id);
            }
        }

        void SetCircleCutDepthInternal(c_id_t const c_id, T cut_depth) {
            assert(c_id < kMaxCircles);
            assert(std::abs(cut_depth) < T{ 0.99 });

            circle_cut_depth_[c_id] = cut_depth;

            if (IsPortalActive(c_id >> 1)) {
                RecalculateVoiceCacheForRemovedCircle(c_id);
                RecalculateVoiceCacheForNewCircle(c_id);
            }
        }

    private:
        void InitPortals() {
            for (p_id_t p_id = 0; p_id < kMaxPortals; ++p_id) {
                circle_axis_[p_id << 1 | 0] = {1.0, 0.0, 0.0};
                circle_axis_[p_id << 1 | 1] = {0.0, 1.0, 0.0};

                circle_cut_depth_[p_id << 1 | 0] = T{ 0.5 };
                circle_cut_depth_[p_id << 1 | 1] = T{ 0.5 };

                SetPortalRotationInternal(p_id, 0.0);
                CalculateCircleUvs(p_id);
            }
        }

        void CalculateCircleUvs(p_id_t const p_id) const {
            assert(p_id < kMaxPortals);

            c_id_t const a_c_id = p_id << 1 | 0;
            c_id_t const b_c_id = p_id << 1 | 1;
            
            auto const c1_axis = circle_axis_[a_c_id];
            auto const c2_axis = circle_axis_[b_c_id];

            if (auto const new_u = c1_axis.Cross(c2_axis); new_u.MagSq() > kParallelEps<T>) {
                portal_u_[p_id] = new_u.Norm();
            }

            circle_v_[a_c_id] = c1_axis.Cross(portal_u_[p_id]).Norm();
            circle_v_[b_c_id] = c2_axis.Cross(portal_u_[p_id]).Norm();
        }

        [[nodiscard]] T Intersect(v_id_t v_id, c_id_t c_id) const {
            assert(v_id < kMaxVoices);
            assert(c_id < kMaxCircles);

            auto const &pos = voice_pos_[v_id];
            auto const &dir = voice_dir_[v_id];

            assert(pos.IsUnit());
            assert(dir.IsUnit());
            assert(std::abs(pos * dir) < kOrthEps<T>);

            Vec3<T> const centre = circle_axis_[c_id];
            T const a = centre * pos;
            T const b = centre * dir;
            T const r_2 = a * a + b * b;

            if (r_2 <= kR2Eps<T>) {
                return std::numeric_limits<T>::max();
            }

            T const normed_cut = circle_cut_depth_[c_id] / std::sqrt(r_2);

            if (std::abs(normed_cut) > T{ 1.0 }) {
                return std::numeric_limits<T>::max();
            }

            auto const intersection_1 = WrapPositiveClosed(std::atan2(b, a) + std::acos(normed_cut), T{ 2.0 } * std::numbers::pi_v<T>);
            auto const intersection_2 = WrapPositiveClosed(std::atan2(b, a) - std::acos(normed_cut), T{ 2.0 } * std::numbers::pi_v<T>);

            return std::min(intersection_1, intersection_2);
        }

        [[nodiscard]] std::tuple<c_id_t, T> FindNextIntersection(v_id_t const v_id) const {
            assert(v_id < kMaxVoices);

            c_id_t current_circle = kInvalidCircle;
            T current_dist{ std::numeric_limits<T>::max() };

            for (size_t i = 0; i < active_portals_; ++i) {
                p_id_t const p_id = portals_[i];
                for (c_id_t c_id = p_id << 1 | 0; c_id <= (p_id << 1 | 1); ++c_id) {
                    if (auto new_dist = Intersect(v_id, c_id);
                        new_dist < current_dist) {
                        current_circle = c_id;
                        current_dist = new_dist;
                    }
                }
            }

            return { current_circle, current_dist };
        }

        void Teleport(v_id_t const v_id, c_id_t const src_id) {
            assert(v_id < kMaxVoices);
            assert(src_id < kMaxCircles);

            p_id_t const p_id = src_id >> 1;
            c_id_t const dst_id = src_id ^ 1;

            assert(IsPortalActive(p_id));

            auto const rot_cos = portal_rot_cos_[p_id];
            auto const rot_sin = (src_id & 1) ? -portal_rot_sin_[p_id] : portal_rot_sin_[p_id];

            auto const &portal_u = portal_u_[p_id];
            auto const &src_v = circle_v_[src_id];
            auto const &dst_v = circle_v_[dst_id];
            auto const src_cut_depth = circle_cut_depth_[src_id];
            auto const src_radius_sq = T{ 1.0 } - src_cut_depth * src_cut_depth;
            auto const dst_cut_depth = circle_cut_depth_[dst_id];
            auto const dst_radius_sq = T{ 1.0 } - dst_cut_depth * dst_cut_depth;

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            auto const src_tangent_u = pos.Cross(circle_axis_[src_id]).Norm();
            auto const src_tangent_v = pos.Cross(src_tangent_u);

            auto const rotated_portal_u = (rot_cos * portal_u - rot_sin * dst_v);
            auto const rotated_portal_v = (rot_sin * portal_u + rot_cos * dst_v);

            pos = dst_cut_depth * circle_axis_[dst_id] +
                  std::sqrt(dst_radius_sq / src_radius_sq) * 
                  (pos * portal_u * rotated_portal_u + pos * src_v * rotated_portal_v);
  
            auto const dst_tangent_u = pos.Cross(circle_axis_[dst_id]).Norm();
            auto const dst_tangent_v = pos.Cross(dst_tangent_u);
            
            dir = dir * src_tangent_u * dst_tangent_u +
                  dir * src_tangent_v * dst_tangent_v;

            // sanitize invariants to avoid error accumulation
            pos = pos.Norm();
            dir = (dir - (dir * pos) * pos).Norm();
        }

        std::array<p_id_t, kMaxPortals> portals_{};
        std::array<T, kMaxPortals> portal_rot_cos_{};
        std::array<T, kMaxPortals> portal_rot_sin_{};
        p_id_t active_portals_{ 0 };

        mutable std::array<Vec3<T>, kMaxPortals> portal_u_{};

        std::array<Vec3<T>, kMaxCircles> circle_axis_{};
        std::array<T, kMaxCircles> circle_cut_depth_{};

        mutable std::array<Vec3<T>, kMaxCircles> circle_v_{};

        // ############################## Terrain #############################

    public:
        void LoadTerrain(std::filesystem::path const &path) {
            Terrain<float, kTextureResolution> new_terrain{ path };

            std::swap(terrain_, new_terrain);
        }

    private:
        Terrain<float, kTextureResolution> terrain_;
    };
    
}
