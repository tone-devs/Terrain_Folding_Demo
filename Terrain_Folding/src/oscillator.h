#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <numbers>
#include <tuple>

#include "globals.h"
#include "terrain.h"
#include "vec.h"

namespace td {

    template<typename T>
        requires (std::floating_point<T>)
    class Oscillator {
    public:
        Oscillator() {
            InitVoices();
            InitPortals();
        }

    private:
        // ############################## Voices ##############################

        using v_id_t = uint8_t;
        static v_id_t constexpr kMaxVoices = 32;

    public:
        void ActivateVoice() {
            if (active_voices_ < kMaxVoices) {
                ++active_voices_;
            }
        }

        void ActivateVoice(Vec3<T> const pos, Vec2<T> const dir, T const freq) {
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

        void SetVoicePos(v_id_t const v_id, Vec3<T> const pos) {
            assert(v_id < kMaxVoices);
            assert(pos.IsUnit());

            voice_pos_[v_id] = pos.Norm();
        }

        void SetVoiceDir(v_id_t const v_id, Vec2<T> const new_dir) {
            assert(v_id < kMaxVoices);

            static T constexpr kMinAngle = static_cast<T>(1e-6);
            static T constexpr kCosEps = static_cast<T>(0.5) * kMinAngle * kMinAngle;

            assert(new_dir.IsUnit());

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            assert(pos.IsUnit());

            Vec3<T> seed_vec;
            if (FloatAbsComparison<kCosEps,T>(pos.Z(), static_cast<T>(1.0))) {
                seed_vec = {0.0, 1.0, 0.0};
            } else if (FloatAbsComparison<kCosEps,T>(pos.Z(), static_cast<T>(-1.0))) {
                seed_vec = {0.0, -1.0, 0.0};
            } else {
                seed_vec = {0.0, 0.0, 1.0};
            }

            Vec3<T> pos_tangent_u = seed_vec.Cross(pos).Norm();
            Vec3<T> pos_tangent_v = pos.Cross(pos_tangent_u).Norm();

            dir = new_dir.X() * pos_tangent_u + new_dir.Y() * pos_tangent_v;

            // sanitize orthonormal
            dir = (dir - (dir * pos) * pos).Norm();
        }

        void SetVoiceFreq(v_id_t const v_id, T const freq) {
            assert(v_id < kMaxVoices);

            voice_freq_[v_id] = freq;
            voice_angular_inc_[v_id] = static_cast<T>(2.0) * std::numbers::pi_v<T> * freq / kSampleRate;
        }

    private:
        void InitVoices() {
            for (v_id_t id = 0; id < kMaxVoices; ++id) {
                voice_pos_[id] = {1.0, 0.0, 0.0};
                voice_dir_[id] = {0.0, 0.0, 1.0};
                SetVoiceFreq(id, static_cast<T>(440.0));
            }
        }

        void Advance(v_id_t const v_id, T const angular_inc) {
            assert(v_id < kMaxVoices);

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            auto new_pos = std::cos(angular_inc) * pos + std::sin(angular_inc) * dir;
            dir = std::cos(angular_inc) * dir - std::sin(angular_inc) * pos;
            pos = new_pos;
        } 

        void AdvanceVoices() {
            for (v_id_t v_id = 0; v_id < active_voices_; ++v_id) {
                for (T angular_inc = voice_angular_inc_[v_id];
                     angular_inc > static_cast<T>(0.0);) {
                    auto [c_id, dist] = FindNextIntersection(v_id);

                    if (dist < angular_inc) {
                        Advance(v_id, dist);
                        angular_inc -= dist;

                        Teleport(v_id, c_id);
                    } else {
                        Advance(v_id, angular_inc);
                        angular_inc = static_cast<T>(0.0);
                    }
                }
            }
        }

        std::array<Vec3<T>, kMaxVoices> voice_pos_{};
        std::array<Vec3<T>, kMaxVoices> voice_dir_{};
        std::array<T, kMaxVoices> voice_freq_{};
        v_id_t active_voices_ = 0;

        mutable std::array<T, kMaxVoices> voice_angular_inc_{};
        
        // ############################## Portals #############################

        using p_id_t = uint8_t;
        using c_id_t = uint8_t;

        static p_id_t constexpr kMaxPortals = 1 << 6; // this keeps c_ids from loosing portal id_bits
        static c_id_t constexpr kMaxCircles = static_cast<c_id_t>(2) * kMaxPortals;
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
        }

        void DeactivatePortal(p_id_t const p_id) {
            assert(IsPortalActive(p_id));
            assert(p_id < kMaxPortals);

            for (size_t i = 0; i < active_portals_; ++i) {
                if (portals_[i] == p_id) {
                    std::swap(portals_[i], portals_[--active_portals_]);
                }
            }
        }

        void SetPortalRotation(p_id_t const p_id, T const rot) {
            assert(p_id < kMaxPortals);

            portal_rotation_[p_id] = rot;
        }

        void SetCircleAxis(c_id_t const c_id, Vec3<T> axis) {
            assert(c_id < kMaxCircles);
            assert(axis.IsUnit());

            circle_axis_[c_id] = axis.Norm();
            CalculateUvs(c_id >> 1);
        }

        void SetCircleCutDepth(c_id_t const c_id, T cut_depth) {
            assert(c_id < kMaxCircles);
            assert(std::abs(cut_depth) < static_cast<T>(0.99));

            circle_cut_depth_[c_id] = cut_depth;
        }

    private:
        void InitPortals() {
            for (p_id_t i = 0; i < kMaxPortals; ++i) {
                circle_axis_[i << 1 | 0] = {1.0, 0.0, 0.0};
                circle_axis_[i << 1 | 1] = {0.0, 1.0, 0.0};

                circle_cut_depth_[i << 1 | 0] = static_cast<T>(0.5);
                circle_cut_depth_[i << 1 | 1] = static_cast<T>(0.5);

                CalculateUvs(i);
            }
        }

        void CalculateUvs(p_id_t const p_id) const {
            static T constexpr kMinAngle = static_cast<T>(1e-6);
            static T constexpr kParallelEps = kMinAngle * kMinAngle;

            assert(p_id < kMaxPortals);

            c_id_t a_c_id = p_id << 1 | 0;
            c_id_t b_c_id = p_id << 1 | 1;
            
            auto c1_axis = circle_axis_[a_c_id];
            auto c2_axis = circle_axis_[b_c_id];

            if (auto new_u = c1_axis.Cross(c2_axis); new_u.MagSq() > kParallelEps) {
                portal_u_[p_id] = new_u.Norm();
            }

            circle_v_[a_c_id] = c1_axis.Cross(portal_u_[p_id]).Norm();
            circle_v_[b_c_id] = c2_axis.Cross(portal_u_[p_id]).Norm();
        }

        [[nodiscard]] T Intersect(v_id_t v_id, c_id_t c_id) const {
            static T constexpr kOrthEps = static_cast<T>(64.0) * std::numeric_limits<T>::epsilon();
            static T constexpr kR2Eps = static_cast<T>(64.0) * std::numeric_limits<T>::epsilon();

            assert(v_id < kMaxVoices);
            assert(c_id < kMaxCircles);

            auto const &pos = voice_pos_[v_id];
            auto const &dir = voice_dir_[v_id];

            assert(pos.IsUnit());
            assert(dir.IsUnit());
            assert(std::abs(pos * dir) < kOrthEps);

            Vec3<T> const centre = circle_axis_[c_id];
            T const a = centre * pos;
            T const b = centre * dir;
            T const r_2 = a * a + b * b;

            if (r_2 <= kR2Eps) {
                return std::numeric_limits<T>::max();
            }

            T const normed_cut = -circle_cut_depth_[c_id] / std::sqrt(r_2);

            if (std::abs(normed_cut) > static_cast<T>(1.0)) {
                return std::numeric_limits<T>::max();
            }

            auto intersection_1 = WrapPositiveClosed(std::atan2(b, a) + std::acos(normed_cut), static_cast<T>(2.0) * std::numbers::pi_v<T>);
            auto intersection_2 = WrapPositiveClosed(std::atan2(b, a) - std::acos(normed_cut), static_cast<T>(2.0) * std::numbers::pi_v<T>);

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

            auto rotation = portal_rotation_[p_id];
            if (src_id & 1) { rotation *= static_cast<T>(-1.0); }

            auto const &portal_u = portal_u_[p_id];
            auto const &src_v = circle_v_[src_id];
            auto const &dst_v = circle_v_[dst_id];
            auto const src_cut_depth = circle_cut_depth_[src_id];
            auto const src_radius_sq = static_cast<T>(1.0) - src_cut_depth * src_cut_depth;
            auto const dst_cut_depth = circle_cut_depth_[dst_id];
            auto const dst_radius_sq = static_cast<T>(1.0) - dst_cut_depth * dst_cut_depth;

            auto &pos = voice_pos_[v_id];
            auto &dir = voice_dir_[v_id];

            auto const src_tangent_u = pos.Cross(circle_axis_[src_id]).Norm();
            auto const src_tangent_v = pos.Cross(src_tangent_u);

            auto const rotated_portal_u = (std::cos(rotation) * portal_u - std::sin(rotation) * dst_v);
            auto const rotated_portal_v = (std::sin(rotation) * portal_u + std::cos(rotation) * dst_v);

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
        std::array<T, kMaxPortals> portal_rotation_{};
        p_id_t active_portals_{ 0 };

        mutable std::array<Vec3<T>, kMaxPortals> portal_u_{};

        std::array<Vec3<T>, kMaxCircles> circle_axis_{};
        std::array<T, kMaxCircles> circle_cut_depth_{};

        mutable std::array<Vec3<T>, kMaxCircles> circle_v_{};

        // ############################## Terrain #############################

        Terrain<T> terrain_;
    };
    
}
