#include <array>
#include <cassert>
#include <numbers>
#include <vector>

#include "globals.h"
#include "utils.h"

//template<template<typename... Ts> typename C, typename... Ts>
//class Soa {
//private:
//    static constexpr size_t kElementCount = sizeof... (Ts);
//    static constexpr std::array<size_t, kElementCount> kOffsetFactor = 
//        []<size_t... indices>(std::index_sequence<indices...>) consteval noexcept {
//            std::array<size_t, kElementCount> temp{};
//
//            (..., (temp[indices + 1] = temp[indices] + FieldSegments::getWidth()));
//
//            return temp;
//        }(std::make_index_sequence<sizeof...(FieldSegments)>{});
//
//public:
//    Soa(std::initializer_list<C<Ts>> const &init) {
//        
//    }
//
//    void Reserve(size_t const size) {
//        
//    }
//
//private:
//
//    std::array<void *, kElementCount> ptrs_{ nullptr };
//    size_t size_{ 0 };
//    size_t capacity_{ 0 };
//};

namespace td {

    template<typename T, size_t order>
        requires (order >= 1)
    struct Vec {
    public:
        [[nodiscard]] T &X() {
            return elems[0];
        }

        [[nodiscard]] T X() const {
            return elems[0];
        }

        [[nodiscard]] T &Y() 
            requires (order >= 2) {
            return elems[1];
        }

        [[nodiscard]] T Y() const
            requires (order >= 2) {
            return elems[1];
        }

        [[nodiscard]] T &Z() 
            requires (order >= 3) {
            return elems[2];
        }

        [[nodiscard]] T Z() const
            requires (order >= 3) {
            return elems[2];
        }

        [[nodiscard]] T &W() 
            requires (order >= 4) {
            return elems[3];
        }

        [[nodiscard]] T W() const
            requires (order >= 4) {
            return elems[3];
        }

        [[nodiscard]] T MagSqrd() const {
            return *this * *this;
        }
        
        [[nodiscard]] T Mag() const {
            return std::sqrt(MagSqrd());
        }

        [[nodiscard]] T &operator[](size_t const &i) {
            assert(i < order);
            return elems[i];
        }

        [[nodiscard]] T operator[](size_t const &i) const {
            assert(i < order);
            return elems[i];
        }

        [[nodiscard]] T operator*(Vec const &o) const {
            T acc{};

            for (size_t i = 0; i < order; ++i) {
                acc += elems[i] * o[i];
            }

            return acc;
        }


        template<size_t start, size_t len>
            requires (start < order && start + len <= order) 
        [[nodiscard]] Vec<T, len> Slice() const {
            Vec<T, len> result{};

            memcpy_s(result.elems, len, elems + start, order - start);

            return result;
        }

        [[nodiscard]] bool IsUnit() const {
            return FloatComparison<std::numeric_limits<T>::epsilon, static_cast<T>(4.0) * std::numeric_limits<T>::epsilon()>(MagSqrd(), static_cast<T>(1.0));
        }
        
        std::array<T, order> elems{};
    };
    
    template<typename T>
    using Vec2 = Vec<T, 2>;

    template<typename T>
    using Vec3 = Vec<T, 3>;

    template<typename T>
    using Vec4 = Vec<T, 4>;

    template<typename T>
        requires( numeric<T> )
    struct Circle {
        Vec4<T> cut_plane;
    };

    template<typename T>
        requires (numeric<T>)
    struct Intersection {
        Circle<T> c{};
        T theta{ std::numeric_limits<T>::max() };
    };

    template<typename T>
        requires (std::floating_point<T>)
    struct Osc {
        Vec3<T> pos;
        Vec3<T> dir;
        T freq;

        [[nodiscard]] Intersection<T> CircleIntersect(Circle<T> &c) const {
            assert(pos.IsUnit());
            assert(dir.IsUnit());
            
            Vec3<T> const centre = c.cut_plane.template Slice<0, 3>();
            T const a = centre * pos;
            T const b = centre * dir;

            T const r_2 = a * a + b * b;
            T const normed_cut = -c.cut_plane[3] / std::sqrt(r_2);

            if (std::abs(normed_cut) > static_cast<T>(1.0)) { return { c }; }

            return { c, std::acos(normed_cut) + std::atan2(b, a) };
        }

        [[nodiscard]] Intersection<T> FindNextIntersection(std::vector<Circle<T>> const &circles) const {
            Intersection<T> current{};

            for (auto &c : circles) {
                if (auto new_i = CircleIntersect(c); new_i.theta < current.theta) { current = new_i; }
            }

            return current;
        }

        T Advance(std::vector<Circle<T>> const &circles) {
            auto next_intersection = FindNextIntersection(circles);

            T angular_inc = static_cast<T>(2.0) * std::numbers::pi_v<T> * freq / kSampleRate;

            if (next_intersection.theta < angular_inc) {
                
            }
        }
    };

    template<typename T>
        requires (numeric<T>)
    struct Portal {
        Circle<T> a, b;

        void Teleport(Osc<T> osc, Circle<T> &entry) const {

        }
    };

}
