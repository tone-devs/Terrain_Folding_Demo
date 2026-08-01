#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <tuple>

#include "globals.h"
#include "terrain.h"
#include "vec.h"

namespace td {

    template<typename T>
        requires (std::floating_point<T>)
    class Oscillator {
    private:
        static constexpr size_t kTextureResolution = (1ull << 12);
    public:
        explicit Oscillator(std::filesystem::path const &terrain_file) {
            InitVoices();
            InitPortals();
            LoadTerrain(terrain_file);
        }

        T GetNextSample() {
            AdvanceVoices();

            T acc{};
            for (v_id_t v_id = 0; v_id < active_voices_; ++v_id) {
                acc += terrain_->ReadPos(voice_pos_[v_id]);
            }

            return acc;
        }

    private:
        // ############################## Voices ##############################

        using v_id_t = uint8_t;
        using p_id_t = uint8_t;
        using c_id_t = uint8_t;

        static v_id_t constexpr kMaxVoices = 32;

    public:
        void ActivateVoice() {
            if (active_voices_ < kMaxVoices) {
                ++active_voices_;
            }
        }

        void ActivateVoice(Vec3<T> const &pos, Vec2<T> const &dir, T const freq) {
            if (active_voices_ < kMaxVoices) {
                SetVoicePos(active_voices_, pos);
                SetVoiceDir(active_voices_, dir);
                SetVoiceFreq(active_voices_, freq);
                ++active_voices_;
            }
        }

        void DeactivateVoice() {
            if (active_voices_ != 0) {
                --active_voices_;
            }
        }

        void SetVoicePos(v_id_t const v_id, Vec3<T> const &new_pos) {
            assert(v_id < kMaxVoices);
            assert(new_pos.IsUnit());

            Vec3<T> &current_pos = voice_pos_[v_id];
            Vec3<T> &current_dir = voice_dir_[v_id];

            auto const [old_u, old_v] = CalculateVoiceUvs(current_pos);
            T u_heading = current_dir * old_u;
            T v_heading = current_dir * old_v;

            current_pos = new_pos.Norm();

            auto const [new_u, new_v] = CalculateVoiceUvs(new_pos);

            current_dir = (u_heading * new_u + v_heading * new_v).Norm();

            RecalculateVoiceCacheForVoice(v_id);
        }

        void SetVoiceDir(v_id_t const v_id, Vec2<T> const &new_dir) {
            assert(v_id < kMaxVoices);
            assert(new_dir.IsUnit());

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            assert(pos.IsUnit());

            auto const [u, v] = CalculateVoiceUvs(pos);
            dir = new_dir.X() * u + new_dir.Y() * v;

            // sanitize orthonormal
            dir = (dir - (dir * pos) * pos).Norm();

            RecalculateVoiceCacheForVoice(v_id);
        }

        void SetVoiceFreq(v_id_t const v_id, T const freq) {
            assert(v_id < kMaxVoices);

            voice_freq_[v_id] = freq;
            voice_angular_inc_[v_id] = T{ 2.0 } * std::numbers::pi_v<T> *freq / kSampleRate;
            voice_step_cos_[v_id] = std::cos(voice_angular_inc_[v_id]);
            voice_step_sin_[v_id] = std::sin(voice_angular_inc_[v_id]);
        }

    private:
        struct Uv { Vec3<T> u, v; };

        static Uv CalculateVoiceUvs(Vec3<T> const &pos) {
            Vec3<T> u = Vec3<T>{0.0, 0.0, 1.0}.Cross(pos);

            if (u.MagSq() <= kParallelEps<T>) {
                Vec3<T> u_fallback = {1.0, 0.0, 0.0};
                u = (u_fallback - (u_fallback * pos) * pos).Norm();
            } else {
                u = u.Norm();
            }

            Vec3<T> v = pos.Cross(u).Norm();

            return { .u = u, .v = v };
        }

        void InitVoices() {
            for (v_id_t id = 0; id < kMaxVoices; ++id) {
                voice_pos_[id] = {1.0, 0.0, 0.0};
                voice_dir_[id] = {0.0, 0.0, 1.0};
                SetVoiceFreq(id, T{ 440.0 });
                
                voice_next_circ_[id] = kInvalidCircle;
                voice_dist_to_circ_[id] = std::numeric_limits<T>::infinity();
            }
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

        std::array<Vec3<T>, kMaxVoices> voice_pos_{};
        std::array<Vec3<T>, kMaxVoices> voice_dir_{};
        std::array<T, kMaxVoices> voice_freq_{};
        v_id_t active_voices_ = 0;

        mutable std::array<T, kMaxVoices> voice_angular_inc_{};
        mutable std::array<c_id_t, kMaxVoices> voice_next_circ_{};
        mutable std::array<T, kMaxVoices> voice_dist_to_circ_{};
        mutable std::array<T, kMaxVoices> voice_step_sin_{};
        mutable std::array<T, kMaxVoices> voice_step_cos_{};
        
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

        void ActivatePortal(p_id_t const p_id) {
            assert(!IsPortalActive(p_id));
            assert(p_id < kMaxPortals);

            portals_[active_portals_++] = p_id;
            RecalculateVoiceCacheForNewCircle(p_id << 1 | 0);
            RecalculateVoiceCacheForNewCircle(p_id << 1 | 1);
        }

        void DeactivatePortal(p_id_t const p_id) {
            assert(IsPortalActive(p_id));
            assert(p_id < kMaxPortals);

            for (size_t i = 0; i < active_portals_; ++i) {
                if (portals_[i] == p_id) {
                    std::swap(portals_[i], portals_[--active_portals_]);
                }
            }

            RecalculateVoiceCacheForRemovedCircle(p_id << 1 | 0);
            RecalculateVoiceCacheForRemovedCircle(p_id << 1 | 1);
        }

        void SetPortalRotation(p_id_t const p_id, T const rot) {
            assert(p_id < kMaxPortals);

            portal_rot_cos_[p_id] = std::cos(rot);
            portal_rot_sin_[p_id] = std::sin(rot);
        }

        void SetCircleAxis(c_id_t const c_id, Vec3<T> axis) {
            assert(c_id < kMaxCircles);
            assert(axis.IsUnit());

            circle_axis_[c_id] = axis.Norm();
            CalculateCircleUvs(c_id >> 1);
            
            if (IsPortalActive(c_id >> 1)) {
                RecalculateVoiceCacheForRemovedCircle(c_id);
                RecalculateVoiceCacheForNewCircle(c_id);
            }
        }

        void SetCircleCutDepth(c_id_t const c_id, T cut_depth) {
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

                SetPortalRotation(p_id, 0.0);
                CalculateCircleUvs(p_id);
            }
        }

        void CalculateCircleUvs(p_id_t const p_id) const {
            assert(p_id < kMaxPortals);

            c_id_t a_c_id = p_id << 1 | 0;
            c_id_t b_c_id = p_id << 1 | 1;
            
            auto const c1_axis = circle_axis_[a_c_id];
            auto const c2_axis = circle_axis_[b_c_id];

            if (auto new_u = c1_axis.Cross(c2_axis); new_u.MagSq() > kParallelEps<T>) {
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

            T const normed_cut = -circle_cut_depth_[c_id] / std::sqrt(r_2);

            if (std::abs(normed_cut) > T{ 1.0 }) {
                return std::numeric_limits<T>::max();
            }

            auto intersection_1 = WrapPositiveClosed(std::atan2(b, a) + std::acos(normed_cut), T{ 2.0 } * std::numbers::pi_v<T>);
            auto intersection_2 = WrapPositiveClosed(std::atan2(b, a) - std::acos(normed_cut), T{ 2.0 } * std::numbers::pi_v<T>);

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

            p_id_t p_id = src_id >> 1;
            c_id_t dst_id = src_id ^ 1;

            assert(IsPortalActive(p_id));

            auto rot_cos = portal_rot_cos_[p_id];
            auto rot_sin = portal_rot_sin_[p_id];
            if (src_id & 1) { rot_sin *= T{ -1.0 }; }

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

        void LoadTerrain(std::filesystem::path const &path) {
            terrain_ = std::make_unique<Terrain<T, kTextureResolution>>(path);
        }

        std::unique_ptr<Terrain<T, kTextureResolution>> terrain_;
    };
    
}
