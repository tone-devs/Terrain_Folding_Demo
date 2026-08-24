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

#include "exchanger.hpp"
#include "globals.hpp"
#include "terrain.hpp"
#include "triple_buffer.hpp"
#include "vec.hpp"

namespace td {

    static_assert(std::atomic_uint8_t::is_always_lock_free);
    
    /// <summary>
    /// Terrain Oscillator defined over S2.  Circular portals can be instantiated on the surface, 
    /// and when a voice crosses a portal, it is transported to the corresponding portal output.
    ///
    /// Expects to be accessed by one control thread, and one audio thread.  
    /// Both threads should be stopped and/or joined before Oscillator is destroyed.
    /// </summary>
    /// <typeparam name="T"></typeparam>

    template<typename T>
        requires (std::floating_point<T>)
    class Oscillator {
    private:
        static constexpr size_t kTextureResolution = (1ull << 12);

    public:
        explicit Oscillator(std::filesystem::path const &terrain_file) : 
            terrain_exchanger_{ std::make_unique<Terrain<float, kTextureResolution>>(terrain_file) } {
            InitVoices();
            InitPortals();
        }

        // AUDIO THREAD ONLY
        void GetNextBlock(std::array<std::array<float, kBlockSize>, 2> &block) {
            CollectVoiceChanges();
            CollectPortalChanges();
            terrain_exchanger_.Consume();
            ApplyVoiceDeactivations();
            bool const voice_cache_dirty = ApplyPortalChanges();
            ApplyActiveVoiceChanges(voice_cache_dirty);

            if (voice_cache_dirty) {
                RecalculateVoiceCaches();
            }

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
                    for (size_t k = 0; k < active_voice_count_; ++k) {
                        block[i][j] += voices_block[j][k] * voice_amp_[i][k];
                    }
                }
            }
        }

    private:
        // ############################## Voices ##############################

        using v_id_t = uint8_t;
        using v_lane_t = uint8_t;

        using p_id_t = uint8_t;
        using p_lane_t = uint8_t;

        using c_id_t = uint8_t;
        using c_lane_t = uint8_t;

        using v_gen_t = uint64_t;
        using p_gen_t = uint64_t;

        static uint8_t constexpr kMaxVoices = 32;
        static v_id_t constexpr kInvalidVoiceId = std::numeric_limits<v_id_t>::max();
        static v_lane_t constexpr kInvalidVoiceLane = std::numeric_limits<v_lane_t>::max();

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
        // CONTROL THREAD ONLY
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

        // CONTROL THREAD ONLY
        [[nodiscard]] bool DeactivateVoice(VoiceHandle const &handle) noexcept {
            if (!IsVoiceActive(handle)) {
                return false;
            }

            auto &current_pending = voice_pending_states_control_[handle.v_id];
            current_pending.active = false;

            voice_state_buffers_[handle.v_id].Publish(current_pending);

            return true;
        }

        // CONTROL THREAD ONLY
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

        // CONTROL THREAD ONLY
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

        // CONTROL THREAD ONLY
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

        // CONTROL THREAD ONLY
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

        // CONTROL THREAD ONLY
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
        void SetVoicePosInternal(v_lane_t const lane, Vec3<T> const &new_pos) {
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
        }

        void SetVoiceDirInternal(v_lane_t const lane, Vec2<T> const &new_dir) {
            assert(lane < kMaxVoices);
            assert(new_dir.IsUnit());

            auto &pos = voice_pos_[lane];
            auto &dir = voice_dir_[lane];

            assert(pos.IsUnit());

            auto const [u, v] = CalculateVoiceUvs(pos);
            dir = new_dir.X() * u + new_dir.Y() * v;

            // sanitize orthonormal
            dir = (dir - (dir * pos) * pos).Norm();
        }

        void SetVoiceFreqInternal(v_lane_t const lane, T const freq) {
            assert(lane < kMaxVoices);
            assert(freq >= 0.0f && freq < kSampleRate / 2 && std::isfinite(freq));

            voice_freq_[lane] = freq;
            voice_angular_inc_[lane] = T{ 2.0 } * std::numbers::pi_v<T> *freq / kSampleRate;
            voice_step_cos_[lane] = std::cos(voice_angular_inc_[lane]);
            voice_step_sin_[lane] = std::sin(voice_angular_inc_[lane]);
        }

        void SetVoiceGainInternal(v_lane_t const lane, float const gain) {
            assert(lane < kMaxVoices);
            assert(gain >= 0.0f && std::isfinite(gain));

            voice_gain_[lane] = gain;
            CalculateAmp(lane);
        }

        void SetVoicePanInternal(v_lane_t const lane, float const pan) {
            assert(lane < kMaxVoices);
            assert(pan >= 0.0f && pan <= 1.0f && std::isfinite(pan));

            voice_pan_[lane] = pan;
            CalculateAmp(lane);
        }

        void CalculateAmp(v_lane_t const lane) const {
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
            for (v_lane_t lane = 0; lane < kMaxVoices; ++lane) {
                voice_pos_[lane] = {1.0, 0.0, 0.0};
                voice_dir_[lane] = {0.0, 0.0, 1.0};
                SetVoiceFreqInternal(lane, T{ 440.0 });
                SetVoicePanInternal(lane, 0.5f);
                
                voice_next_circ_[lane] = kInvalidCircleLane;
                voice_dist_to_circ_[lane] = std::numeric_limits<T>::infinity();
            }

            voice_slot_to_lane_.fill(kInvalidVoiceLane);
            voice_lane_to_slot_.fill(kInvalidVoiceId);
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

                if (v_lane_t const lane = voice_slot_to_lane_[v_id]; lane != kInvalidVoiceLane) {
                    v_lane_t const last_lane = --active_voice_count_;
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

                        v_id_t const moved_v_id = voice_lane_to_slot_[last_lane];
                        voice_slot_to_lane_[moved_v_id] = lane;
                        voice_lane_to_slot_[lane] = moved_v_id;
                    }

                    voice_lane_to_slot_[last_lane] = kInvalidVoiceId;
                    voice_slot_to_lane_[v_id] = kInvalidVoiceLane;
                }

                auto &current_state = voice_state_[v_id];
                auto const &new_state = voice_pending_states_audio_[v_id];
                current_state.gen = new_state.gen;
                current_state.param_gens = new_state.param_gens;

                voice_pending_flags_[v_id] = false;
            }
        }

        void ApplyActiveVoiceChanges(bool const voice_cache_dirty) {
            for (v_id_t v_id = 0; v_id < kMaxVoices; ++v_id) {
                if (!voice_pending_flags_[v_id]) {
                    continue;
                }

                auto const &pending = voice_pending_states_audio_[v_id];
                auto &current = voice_state_[v_id];

                if (v_lane_t &lane = voice_slot_to_lane_[v_id]; lane == kInvalidVoiceLane || pending.gen != current.gen) {
                    if (lane == kInvalidVoiceLane) {
                        assert(active_voice_count_ < kMaxVoices);

                        lane = active_voice_count_++;
                        voice_lane_to_slot_[lane] = v_id;
                    }

                    SetVoicePosInternal(lane, pending.params.pos);
                    SetVoiceDirInternal(lane, pending.params.dir);
                    SetVoiceFreqInternal(lane, pending.params.freq);
                    SetVoiceGainInternal(lane, pending.params.gain);
                    SetVoicePanInternal(lane, pending.params.pan);

                    if (!voice_cache_dirty) {
                        RecalculateVoiceCacheForVoice(lane);
                    }

                    current.gen = pending.gen;
                } else {
                    if (!pending.active) {
                        continue;
                    }

                    bool path_changed = false;

                    if (pending.param_gens.pos != current.param_gens.pos) {
                        SetVoicePosInternal(lane, pending.params.pos);
                        path_changed = true;
                    }

                    if (pending.param_gens.dir != current.param_gens.dir) {
                        SetVoiceDirInternal(lane, pending.params.dir);
                        path_changed = true;
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

                    if (path_changed && !voice_cache_dirty) {
                        RecalculateVoiceCacheForVoice(lane);
                    }
                }
                current.param_gens = pending.param_gens;
                voice_pending_flags_[v_id] = false;
            }
        }

        [[nodiscard]] std::array<float, kMaxVoices> GetVoicesSamples() const {
            std::array<float, kMaxVoices> samples{};
            auto & terrain = terrain_exchanger_.Current();
            for (size_t i = 0; i < active_voice_count_; ++i) {
                samples[i] = terrain.ReadPos(voice_pos_[i]);
            }
            return samples;
        }

        void AdvanceVoice(v_lane_t const lane) {
            AdvanceVoice(lane, voice_step_cos_[lane], voice_step_sin_[lane]);
        } 

        void AdvanceVoice(v_lane_t const lane, T const angular_inc) {
            AdvanceVoice(lane, std::cos(angular_inc), std::sin(angular_inc));
        }

        void AdvanceVoice(v_lane_t const lane, T const cos_theta, T const sin_theta) {
            assert(lane < kMaxVoices);

            auto &pos = voice_pos_[lane];
            auto &dir = voice_dir_[lane];

            auto new_pos = cos_theta * pos + sin_theta * dir;
            dir = cos_theta * dir - sin_theta * pos;
            pos = new_pos.Norm();
            dir = (dir - (dir * pos) * pos).Norm();
        }

        void AdvanceVoices() {
            for (v_lane_t lane = 0; lane < active_voice_count_; ++lane) {
                T angular_inc = voice_angular_inc_[lane];
                c_lane_t next_circ = voice_next_circ_[lane];
                T dist_to_circ = voice_dist_to_circ_[lane];

                if (dist_to_circ > angular_inc) {
                    AdvanceVoice(lane);
                    dist_to_circ -= angular_inc;
                    angular_inc = T{ 0.0 };
                } else {
                    HandlePortalCrossing(lane, angular_inc, next_circ, dist_to_circ);
                }

                voice_next_circ_[lane] = next_circ;
                voice_dist_to_circ_[lane] = dist_to_circ;
            }
        }

        void HandlePortalCrossing(v_lane_t const v_lane, T angular_inc, c_lane_t &next_c_lane, T &dist_to_circ) {
            while (angular_inc > T{ 0.0 }) {
                if (dist_to_circ > angular_inc) {
                    AdvanceVoice(v_lane, angular_inc);
                    dist_to_circ -= angular_inc;
                    angular_inc = T{ 0.0 };
                } else {
                    AdvanceVoice(v_lane, dist_to_circ);
                    angular_inc -= dist_to_circ;

                    Teleport(v_lane, next_c_lane);

                    std::tie(next_c_lane, dist_to_circ) = FindNextIntersection(v_lane);
                }
            }
        }
        
        void RecalculateVoiceCacheForVoice(v_lane_t const lane) const {
            std::tie(voice_next_circ_[lane], voice_dist_to_circ_[lane]) = FindNextIntersection(lane);
        }

        void RecalculateVoiceCaches() const {
            for (v_lane_t lane = 0; lane < active_voice_count_; ++lane) {
                RecalculateVoiceCacheForVoice(lane);
            }
        }

        // State
        std::array<Vec3<T>, kMaxVoices> voice_pos_{};
        std::array<Vec3<T>, kMaxVoices> voice_dir_{};
        std::array<T, kMaxVoices> voice_freq_{};
        std::array<float, kMaxVoices> voice_gain_{};
        std::array<float, kMaxVoices> voice_pan_{};
        uint8_t active_voice_count_ = 0;

        // Threading
        std::array<VoicePendingState, kMaxVoices> voice_pending_states_control_{};
        std::array<TripleBuffer<VoicePendingState>, kMaxVoices> voice_state_buffers_{};
        std::array<VoicePendingState, kMaxVoices> voice_pending_states_audio_{};
        std::bitset<kMaxVoices> voice_pending_flags_{};
        std::array<VoiceState, kMaxVoices> voice_state_{};
        std::array<v_lane_t, kMaxVoices> voice_slot_to_lane_{};
        std::array<v_id_t, kMaxVoices> voice_lane_to_slot_{};

        // Caching
        mutable std::array<T, kMaxVoices> voice_angular_inc_{};
        mutable std::array<c_lane_t, kMaxVoices> voice_next_circ_{};
        mutable std::array<T, kMaxVoices> voice_dist_to_circ_{};
        mutable std::array<T, kMaxVoices> voice_step_sin_{};
        mutable std::array<T, kMaxVoices> voice_step_cos_{};
        mutable std::array<std::array<float, kMaxVoices>, 2> voice_amp_{};
        
        // ############################## Portals #############################

        static uint8_t constexpr kMaxPortals = 1 << 6; // this keeps c_ids from loosing portal id_bits
        static uint8_t constexpr kMaxCircles = c_id_t{ 2 } * kMaxPortals;
        static c_id_t constexpr kInvalidCircleId = std::numeric_limits<c_id_t>::max();
        static c_lane_t constexpr kInvalidCircleLane = std::numeric_limits<c_lane_t>::max();

        struct PortalParamPack {
            T rotation{};
            std::array<Vec3<T>, 2> axes{};
            std::array<T, 2> cut_depths{};
            bool active{};
        };

        struct PortalParamGenerations {
            uint64_t rotation{};
            std::array<uint64_t, 2> axes{};
            std::array<uint64_t, 2> cut_depth{};
            uint64_t active{};
        };

        struct PortalState {
            PortalParamGenerations param_gens;
        };

        struct PendingPortalState {
            PortalParamPack params{};
            PortalParamGenerations param_gens{};
        };

    public:
        // CONTROL THREAD ONLY
        bool IsPortalActive(p_id_t const p_id) const {
            assert(p_id < kMaxPortals);

            return portal_pending_state_control_[p_id].params.active;
        }

        // CONTROL THREAD ONLY
        bool SetPortalActive(p_id_t const p_id, bool const active) noexcept {
            assert(p_id < kMaxPortals);

            if (active == IsPortalActive(p_id)) {
                return false;
            }

            auto &current_pending = portal_pending_state_control_[p_id];
            current_pending.params.active = active;
            IncGen(current_pending.param_gens.active);

            portal_state_buffers_[p_id].Publish(current_pending);

            return true;
        }

        // CONTROL THREAD ONLY
        bool SetPortalRotation(p_id_t const p_id, T const rot) noexcept {
            assert(p_id < kMaxPortals);

            if (rot < T{ 0.0 } || rot >= T{ 2.0 } * std::numbers::pi_v<T> || !std::isfinite(rot)) {
                return false;
            }

            auto &current_pending = portal_pending_state_control_[p_id];
            current_pending.params.rotation = rot;
            IncGen(current_pending.param_gens.rotation);

            portal_state_buffers_[p_id].Publish(current_pending);

            return true;
        }

        // CONTROL THREAD ONLY
        bool SetCircleAxis(c_id_t const c_id, Vec3<T> axis) noexcept {
            assert(c_id < kMaxCircles);

            if (!axis.IsUnit()) {
                return false;
            }

            p_id_t p_id = c_id >> 1;
            size_t circle_index = c_id & 1;

            auto &current_pending = portal_pending_state_control_[p_id];
            current_pending.params.axes[circle_index] = axis;
            IncGen(current_pending.param_gens.axes[circle_index]);

            portal_state_buffers_[p_id].Publish(current_pending);

            return true;
        }

        // CONTROL THREAD ONLY
        bool SetCircleCutDepth(c_id_t const c_id, T cut_depth) noexcept {
            assert(c_id < kMaxCircles);

            if (std::abs(cut_depth) >= T{ 0.99 } || !std::isfinite(cut_depth)) {
                return false;
            }

            p_id_t p_id = c_id >> 1;
            size_t circle_index = c_id & 1;

            auto &current_pending = portal_pending_state_control_[p_id];
            current_pending.params.cut_depths[circle_index] = cut_depth;
            IncGen(current_pending.param_gens.cut_depth[circle_index]);

            portal_state_buffers_[p_id].Publish(current_pending);

            return true;
        }

    private:
        void CollectPortalChanges() noexcept {
            for (p_id_t p_id = 0; p_id < kMaxPortals; ++p_id) {
                if (auto pending = portal_state_buffers_[p_id].Consume()) {
                    portal_pending_state_audio_[p_id] = *pending;
                    portal_pending_flags_[p_id] = true;
                }
            }
        }

        bool ApplyPortalChanges() {
            bool geometry_changed = false;
            for (p_id_t p_id = 0; p_id < kMaxPortals; ++p_id) {
                if (!portal_pending_flags_[p_id]) {
                    continue;
                }

                auto const &pending = portal_pending_state_audio_[p_id];
                auto &current = portal_states_[p_id];

                if (pending.param_gens.active != current.param_gens.active) {
                    geometry_changed |= SetPortalActiveInternal(portal_slot_to_lane_[p_id], pending.params.active);
                }

                p_lane_t lane = portal_slot_to_lane_[p_id];

                for (size_t i = 0; i < 2; ++i) {
                    if (pending.param_gens.axes[i] != current.param_gens.axes[i]) {
                        SetCircleAxisInternal(lane << 1 | i, pending.params.axes[i]);
                        geometry_changed |= pending.params.active;
                    }
                }

                for (size_t i = 0; i < 2; ++i) {
                    if (pending.param_gens.cut_depth[i] != current.param_gens.cut_depth[i]) {
                        SetCircleCutDepthInternal(lane << 1 | i, pending.params.cut_depths[i]);
                        geometry_changed |= pending.params.active;
                    }
                }

                if (pending.param_gens.rotation != current.param_gens.rotation) {
                    SetPortalRotationInternal(lane, pending.params.rotation);
                }

                current.param_gens = pending.param_gens;
                portal_pending_flags_[p_id] = false;
            }
            
            return geometry_changed;
        }

        bool IsPortalActiveInternal(p_lane_t const lane) const {
            assert(lane < kMaxPortals);

            return lane < active_portal_count_;
        }

        bool SetPortalActiveInternal(p_lane_t const lane, bool const active) {
            if (bool const currently_active = lane < active_portal_count_; active == currently_active) {
                return false;
            }

            if (active) {
                SwapPortalLanes(lane, active_portal_count_);
                ++active_portal_count_;
            } else {
                --active_portal_count_;
                SwapPortalLanes(lane, active_portal_count_);
            }

            return true;
        }

        void SetPortalRotationInternal(p_lane_t const lane, T const rot) {
            assert(lane < kMaxPortals);

            portal_rot_cos_[lane] = std::cos(rot);
            portal_rot_sin_[lane] = std::sin(rot);
        }

        void SetCircleAxisInternal(c_lane_t const lane, Vec3<T> axis) {
            assert(lane < kMaxCircles);
            assert(axis.IsUnit());

            circle_axis_[lane] = axis.Norm();
            CalculateCircleUvs(lane >> 1);
        }

        void SetCircleCutDepthInternal(c_lane_t const lane, T cut_depth) {
            assert(lane < kMaxCircles);
            assert(std::abs(cut_depth) < T{ 0.99 });

            circle_cut_depth_[lane] = cut_depth;
        }

        void InitPortals() {
            p_id_t p_id = 0;
            p_lane_t p_lane = 0;
            for (; p_id < kMaxPortals && p_lane < kMaxPortals; ++p_id, ++p_lane) {
                portal_slot_to_lane_[p_id] = p_lane;
                portal_lane_to_slot_[p_lane] = p_id;

                circle_axis_[p_lane << 1 | 0] = {1.0, 0.0, 0.0};
                circle_axis_[p_lane << 1 | 1] = {0.0, 1.0, 0.0};

                circle_cut_depth_[p_lane << 1 | 0] = T{ 0.5 };
                circle_cut_depth_[p_lane << 1 | 1] = T{ 0.5 };

                SetPortalRotationInternal(p_lane, 0.0);
                CalculateCircleUvs(p_lane);
            }
        }

        void SwapPortalLanes(p_lane_t const lane_a, p_lane_t const lane_b) {
            if (lane_a == lane_b) { return; }

            std::swap(portal_rot_sin_[lane_a], portal_rot_sin_[lane_b]);
            std::swap(portal_rot_cos_[lane_a], portal_rot_cos_[lane_b]);
            std::swap(portal_u_[lane_a], portal_u_[lane_b]);
            for (size_t i = 0; i < 2; ++i) {
                std::swap(circle_axis_[lane_a << 1 | i], circle_axis_[lane_b << 1 | i]);
            }
            for (size_t i = 0; i < 2; ++i) {
                std::swap(circle_cut_depth_[lane_a << 1 | i], circle_cut_depth_[lane_b << 1 | i]);
            }
            for (size_t i = 0; i < 2; ++i) {
                std::swap(circle_v_[lane_a << 1 | i], circle_v_[lane_b << 1 | i]);
            }
                            
            p_id_t &p_id_a = portal_lane_to_slot_[lane_a];
            p_id_t &p_id_b = portal_lane_to_slot_[lane_b];
            std::swap(p_id_a, p_id_b);
            portal_slot_to_lane_[p_id_b] = lane_b;
            portal_slot_to_lane_[p_id_a] = lane_a;
        }

        void CalculateCircleUvs(p_lane_t const lane) const {
            assert(lane < kMaxPortals);

            c_lane_t const a_c_lane = lane << 1 | 0;
            c_lane_t const b_c_lane = lane << 1 | 1;
            
            auto const c1_axis = circle_axis_[a_c_lane];
            auto const c2_axis = circle_axis_[b_c_lane];

            if (auto const new_u = c1_axis.Cross(c2_axis); new_u.MagSq() > kParallelEps<T>) {
                portal_u_[lane] = new_u.Norm();
            }

            circle_v_[a_c_lane] = c1_axis.Cross(portal_u_[lane]).Norm();
            circle_v_[b_c_lane] = c2_axis.Cross(portal_u_[lane]).Norm();
        }

        [[nodiscard]] T Intersect(v_lane_t const v_lane, c_lane_t const c_lane) const {
            assert(v_lane < kMaxVoices);
            assert(c_lane < kMaxCircles);

            auto const &pos = voice_pos_[v_lane];
            auto const &dir = voice_dir_[v_lane];

            assert(pos.IsUnit());
            assert(dir.IsUnit());
            assert(std::abs(pos * dir) < kOrthEps<T>);

            Vec3<T> const centre = circle_axis_[c_lane];
            T const a = centre * pos;
            T const b = centre * dir;
            T const r_2 = a * a + b * b;

            if (r_2 <= kR2Eps<T>) {
                return std::numeric_limits<T>::max();
            }

            T const normed_cut = circle_cut_depth_[c_lane] / std::sqrt(r_2);

            if (std::abs(normed_cut) > T{ 1.0 }) {
                return std::numeric_limits<T>::max();
            }

            auto const intersection_1 = WrapPositiveClosed(std::atan2(b, a) + std::acos(normed_cut), T{ 2.0 } * std::numbers::pi_v<T>);
            auto const intersection_2 = WrapPositiveClosed(std::atan2(b, a) - std::acos(normed_cut), T{ 2.0 } * std::numbers::pi_v<T>);

            return std::min(intersection_1, intersection_2);
        }

        [[nodiscard]] std::tuple<c_lane_t, T> FindNextIntersection(v_lane_t const v_lane) const {
            assert(v_lane < kMaxVoices);

            c_lane_t current_circle = kInvalidCircleLane;
            T current_dist{ std::numeric_limits<T>::max() };

            for (c_lane_t c_lane = 0; c_lane < 2 * active_portal_count_; ++c_lane) {
                if (auto new_dist = Intersect(v_lane, c_lane);
                    new_dist < current_dist) {
                    current_circle = c_lane;
                    current_dist = new_dist;
                }
            }

            return { current_circle, current_dist };
        }

        void Teleport(v_lane_t const v_lane, c_lane_t const src_c_lane) {
            assert(v_lane < kMaxVoices);
            assert(src_c_lane < kMaxCircles);

            p_lane_t const p_lane = src_c_lane >> 1;
            c_lane_t const dst_c_lane = src_c_lane ^ 1;

            assert(IsPortalActiveInternal(p_lane));

            auto const rot_cos = portal_rot_cos_[p_lane];
            auto const rot_sin = (src_c_lane & 1) ? -portal_rot_sin_[p_lane] : portal_rot_sin_[p_lane];

            auto const &portal_u = portal_u_[p_lane];
            auto const &src_v = circle_v_[src_c_lane];
            auto const &dst_v = circle_v_[dst_c_lane];
            auto const src_cut_depth = circle_cut_depth_[src_c_lane];
            auto const src_radius_sq = T{ 1.0 } - src_cut_depth * src_cut_depth;
            auto const dst_cut_depth = circle_cut_depth_[dst_c_lane];
            auto const dst_radius_sq = T{ 1.0 } - dst_cut_depth * dst_cut_depth;

            auto &pos = voice_pos_[v_lane];
            auto &dir = voice_dir_[v_lane];

            auto const src_tangent_u = pos.Cross(circle_axis_[src_c_lane]).Norm();
            auto const src_tangent_v = pos.Cross(src_tangent_u);

            auto const rotated_portal_u = (rot_cos * portal_u - rot_sin * dst_v);
            auto const rotated_portal_v = (rot_sin * portal_u + rot_cos * dst_v);

            pos = dst_cut_depth * circle_axis_[dst_c_lane] +
                  std::sqrt(dst_radius_sq / src_radius_sq) * 
                  (pos * portal_u * rotated_portal_u + pos * src_v * rotated_portal_v);
  
            auto const dst_tangent_u = pos.Cross(circle_axis_[dst_c_lane]).Norm();
            auto const dst_tangent_v = pos.Cross(dst_tangent_u);
            
            dir = dir * src_tangent_u * dst_tangent_u +
                  dir * src_tangent_v * dst_tangent_v;

            // sanitize invariants to avoid error accumulation
            pos = pos.Norm();
            dir = (dir - (dir * pos) * pos).Norm();
        }

        // State
        std::array<T, kMaxPortals> portal_rot_cos_{};
        std::array<T, kMaxPortals> portal_rot_sin_{};
        std::array<Vec3<T>, kMaxCircles> circle_axis_{};
        std::array<T, kMaxCircles> circle_cut_depth_{};
        uint8_t active_portal_count_{ 0 };

        // Threading
        std::array<PortalState, kMaxPortals> portal_states_;
        std::array<PendingPortalState, kMaxPortals> portal_pending_state_control_{};
        std::array<TripleBuffer<PendingPortalState>, kMaxPortals> portal_state_buffers_{};
        std::array<PendingPortalState, kMaxPortals> portal_pending_state_audio_{};
        std::bitset<kMaxPortals> portal_pending_flags_{};
        std::array<p_id_t, kMaxPortals> portal_lane_to_slot_{};
        std::array<p_lane_t, kMaxPortals> portal_slot_to_lane_{};

        // Cache
        mutable std::array<Vec3<T>, kMaxPortals> portal_u_{};
        mutable std::array<Vec3<T>, kMaxCircles> circle_v_{};

        // ############################## Terrain #############################

    public:
        void LoadTerrain(std::filesystem::path const &path) {
            terrain_exchanger_.Publish(std::make_unique<Terrain<float, kTextureResolution>>(path));
        }

    private:
        Exchanger<Terrain<float, kTextureResolution>> terrain_exchanger_;
    };
    
}
