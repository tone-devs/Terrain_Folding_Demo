#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>

#include"vec.h"

namespace td {

    // temporary implementation till better interpolation figured out
    template<typename T, size_t order>
        requires (std::floating_point<T> &&
                  order >= 2)
    class Terrain 
    {
    public:

        T ReadPos(Vec3<T> const &pos) const {
            static size_t constexpr kStride = order + 1;
            static T constexpr kMaxCoord = T{ order };

            Vec2<T> texture_pos = (OctahedralEncode(pos) + T{ 1.0 }) * T{ 0.5 } * T{ kMaxCoord };
            
            T const x = std::clamp(texture_pos.X(), T{ 0.0 }, kMaxCoord);
            T const y = std::clamp(texture_pos.Y(), T{ 0.0 }, kMaxCoord);

            size_t x_i = std::min(static_cast<size_t>(x), order - 1);
            size_t y_i = std::min(static_cast<size_t>(y), order - 1);

            T x_f = x - static_cast<T>(x_i), y_f = y - static_cast<T>(y_i);

            T p_0_0 = texture_[(y_i    ) * kStride + (x_i    )];
            T p_0_1 = texture_[(y_i    ) * kStride + (x_i + 1)];
            T p_1_0 = texture_[(y_i + 1) * kStride + (x_i    )];
            T p_1_1 = texture_[(y_i + 1) * kStride + (x_i + 1)];

            T p_0_x = std::lerp(p_0_0, p_0_1, x_f);
            T p_1_x = std::lerp(p_1_0, p_1_1, x_f);

            T p_y_x = std::lerp(p_0_x, p_1_x, y_f);

            return p_y_x;
        }

    private:
        std::array<T, (order+1) * (order + 1)> texture_{};
    };
    
}

