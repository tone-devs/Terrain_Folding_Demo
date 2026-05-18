#pragma once
#include <complex>

namespace td {

    template<typename T>
    concept numeric<T> = std::floating_point<T> || std::integral<T>;

    template<auto epsilon, typename T = decltype(epsilon)>
        requires (std::floating_point<T>)
    inline bool FloatAbsComparison(T const &a, T const &b) {
        return std::abs(a - b) < epsilon;
    }

    template<typename T>
        requires (std::floating_point<T>)
    inline bool FloatAbsComparison(T const &a, T const &b, T const &epsilon) {
        return std::abs(a - b) < epsilon;
    }

    template<auto epsilon, typename T = decltype(epsilon)>
        requires (std::floating_point<T>)
    inline bool FloatRelComparison(T const &a, T const &b) {
        return std::abs(a - b) <= epsilon * std::max(std::abs(a), std::abs(b));
    }

    template<typename T>
        requires (std::floating_point<T>)
    inline bool FloatRelComparison(T const &a, T const &b, T const &epsilon) {
        return std::abs(a - b) <= epsilon * std::max(std::abs(a), std::abs(b));
    }

    template<auto abs_epsilon, auto rel_epsilon, typename T = std::common_type_t<decltype(abs_epsilon), decltype(rel_epsilon)>>
        requires (std::floating_point<T>)
    inline bool FloatComparison(T const &a, T const &b) {
        return FloatAbsComparison<abs_epsilon>(a, b) ||
               FloatRelComparison<rel_epsilon>(a, b);
    }

    template<typename T>
        requires (std::floating_point<T>)
    inline bool FloatComparison(T const &a, T const &b, T const &abs_epsilon, T const &rel_epsilon) {
        return FloatAbsComparison(a, b, abs_epsilon) ||
               FloatRelComparison(a, b, rel_epsilon);
    }
    
}
