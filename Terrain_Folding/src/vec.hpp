#pragma once

#include "globals.hpp"
#include"utils.hpp"

namespace td {

    template<typename T, size_t order>
        requires (order >= 1)
    struct Vec {
    public:
        constexpr Vec() = default;

        template<typename... Ts>
            requires(sizeof...(Ts) == order && (std::is_convertible_v<Ts, T> && ...))
        constexpr Vec(Ts&&... args) : elems{ static_cast<T>(std::forward<Ts>(args))... } {}

        template<size_t order_a>
        constexpr Vec(Vec<T, order_a> const& a, Vec<T, order - order_a> const& b) {
            static_assert(order_a < order, "First vector order must be less than total order");
            for (size_t i = 0; i < order_a; ++i) {
                elems[i] = a[i];
            }
            for (size_t i = 0; i < order - order_a; ++i) {
                elems[order_a + i] = b[i];
            }
        }

        template<size_t source_order>
            requires(source_order + 1 == order)
        constexpr Vec(Vec<T, source_order> const &a, T const &b) {
            for (size_t i = 0; i < source_order; ++i) {
                elems[i] = a[i];
            }
            elems[order - 1] = b;
        }

        [[nodiscard]] constexpr T &X() {
            return elems[0];
        }

        [[nodiscard]] constexpr T X() const {
            return elems[0];
        }

        [[nodiscard]] constexpr T &Y()
            requires (order >= 2) {
            return elems[1];
        }

        [[nodiscard]] constexpr T Y() const
            requires (order >= 2) {
            return elems[1];
        }

        [[nodiscard]] constexpr T &Z()
            requires (order >= 3) {
            return elems[2];
        }

        [[nodiscard]] constexpr T Z() const
            requires (order >= 3) {
            return elems[2];
        }

        [[nodiscard]] constexpr T &W()
            requires (order >= 4) {
            return elems[3];
        }

        [[nodiscard]] constexpr T W() const
            requires (order >= 4) {
            return elems[3];
        }

        [[nodiscard]] constexpr Vec<T, 2> XY() const
            requires (order >= 2) {
            return { X(), Y() };
        }

        [[nodiscard]] constexpr Vec<T, 2> YX() const
            requires (order >= 2) {
            return { Y(), X() };
        }

        [[nodiscard]] constexpr T MagSq() const {
            return *this * *this;
        }

        [[nodiscard]] constexpr T Mag() const {
            return std::sqrt(MagSq());
        }

        [[nodiscard]] constexpr T L1() const {
            T acc{};
            
            for (T const &elem : elems) {
                acc += std::abs(elem);
            }

            return acc;
        }

        [[nodiscard]] constexpr T &operator[](size_t const &i) {
            assert(i < order);
            return elems[i];
        }

        [[nodiscard]] constexpr T operator[](size_t const &i) const {
            assert(i < order);
            return elems[i];
        }

        [[nodiscard]] constexpr Vec operator-() const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = -(*this)[i];
            }

            return result;
        }

        [[nodiscard]] constexpr Vec operator+(Vec const &o) const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = (*this)[i] + o[i];
            }

            return result;
        }

        [[nodiscard]] constexpr Vec operator+(T const &x) const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = (*this)[i] + x;
            }

            return result;
        }

        [[nodiscard]] constexpr Vec operator-(Vec const &o) const {
            return *this + -o;
        }

        [[nodiscard]] constexpr Vec operator-(T const &x) const {
            return *this + -x;
        }

        [[nodiscard]] friend constexpr Vec operator-(T const x, Vec const &v) {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result.elems[i] = x - v.elems[i];
            }

            return result;
        }

        [[nodiscard]] constexpr T operator*(Vec const &o) const {
            T acc{};

            for (size_t i = 0; i < order; ++i) {
                acc += elems[i] * o[i];
            }

            return acc;
        }

        [[nodiscard]] constexpr Vec HadamardProd(Vec const &o) const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = elems[i] * o[i];
            }

            return result;
        }

        [[nodiscard]] constexpr Vec operator*(T const &scalar) const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = scalar * elems[i];
            }

            return result;
        }

        [[nodiscard]] friend constexpr Vec operator*(T const &scalar, Vec const &vec) {
            return vec * scalar;
        }

        [[nodiscard]] constexpr Vec operator/(T const &scalar) const {
            return *this * (T{ 1.0 } / scalar);
        }

        [[nodiscard]] constexpr Vec Cross(Vec const &o) const
            requires (order == 3/* || order == 7*/) {
            return { elems[1] * o[2] - elems[2] * o[1],
                     elems[2] * o[0] - elems[0] * o[2],
                     elems[0] * o[1] - elems[1] * o[0] };
        }

        [[nodiscard]] constexpr Vec Norm() const {
            return *this / Mag();
        }

        [[nodiscard]] constexpr bool IsUnit() const {
            static T constexpr kUnitEps = T{ 64.0 } * std::numeric_limits<T>::epsilon();
            return FloatComparison<kUnitEps, kUnitEps>(MagSq(), T{ 1.0 });
        }

        [[nodiscard]] constexpr Vec Sign() const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = elems[i] >= T{ 0.0 } ? T{ 1.0 } : T{ -1.0 };
            }

            return result;
        }

        [[nodiscard]] constexpr Vec Abs() const {
            Vec result{};

            for (size_t i = 0; i < order; ++i) {
                result[i] = std::abs(elems[i]);
            }

            return result;
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
        requires (std::floating_point<T>)
    [[nodiscard]] constexpr Vec2<T> OctahedralEncode(Vec3<T> const &vec) {
        Vec2<T> normed_vec = vec.XY() / vec.L1();
        if (vec.Z() >= T{ 0.0 }) {
            return normed_vec;
        } else {
            return (T{ 1.0 } - normed_vec.Abs()).YX().HadamardProd(normed_vec.Sign());
        }
    }

}
