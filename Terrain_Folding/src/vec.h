#pragma once

#include "globals.h"
#include"utils.h"

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

        [[nodiscard]] T MagSq() const {
            return *this * *this;
        }

        [[nodiscard]] T Mag() const {
            return std::sqrt(MagSq());
        }

        [[nodiscard]] T &operator[](size_t const &i) {
            assert(i < order);
            return elems[i];
        }

        [[nodiscard]] T operator[](size_t const &i) const {
            assert(i < order);
            return elems[i];
        }

        [[nodiscard]] Vec operator-() const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = -(*this)[i];
            }

            return result;
        }

        [[nodiscard]] Vec operator+(Vec const &o) const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = *(this[i]) + o[i];
            }

            return result;
        }

        [[nodiscard]] Vec operator-(Vec const &o) const {
            return *this + -o;
        }

        [[nodiscard]] T operator*(Vec const &o) const {
            T acc{};

            for (size_t i = 0; i < order; ++i) {
                acc += elems[i] * o[i];
            }

            return acc;
        }

        [[nodiscard]] Vec operator*(T const &scalar) const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = scalar * elems[i];
            }

            return result;
        }

        [[nodiscard]] friend Vec operator*(T const &scalar, Vec const &vec) {
            return vec * scalar;
        }

        [[nodiscard]] Vec operator/(T const &scalar) const {
            return *this * (static_cast<T>(1.0) / scalar);
        }

        Vec Cross(Vec const &o) const
            requires (order == 3 || order == 7) {
            return { elems[1] * o[2] - elems[2] * o[1],
                     elems[2] * o[0] - elems[0] * o[2],
                     elems[0] * o[1] - elems[1] * o[0] };
        }

        Vec Norm() const {
            return *this / Mag();
        }

        [[nodiscard]] bool IsUnit() const {
            static T constexpr kUnitEps = static_cast<T>(64.0) * std::numeric_limits<T>::epsilon();
            return FloatComparison<kUnitEps, kUnitEps>(Magsq(), static_cast<T>(1.0));
        }

        std::array<T, order> elems{};
    };

    template<typename T>
    using Vec2 = Vec<T, 2>;

    template<typename T>
    using Vec3 = Vec<T, 3>;

    template<typename T>
    using Vec4 = Vec<T, 4>;

}
