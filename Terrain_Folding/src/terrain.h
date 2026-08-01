#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <filesystem>

#include"stb_image.h"
#include"vec.h"

namespace td {

    // temporary implementation till better interpolation figured out
    template<typename T, size_t resolution>
        requires (std::floating_point<T> &&
                  resolution >= 2)
    class Terrain 
    {
    private:
            static size_t constexpr kStride = resolution;
            static T constexpr kMaxCoord = T{ resolution - 1 };

    public:

        Terrain(std::filesystem::path const &file) {
            LoadTexture(file);
        }

        void LoadTexture(std::filesystem::path const &file) {
            int x, y, n;

            std::string const file_name = file.string();

            if (!stbi_is_16_bit(file_name.c_str())) {
                throw std::runtime_error("Terrain image must be 16-bit: " + file.string());
            }

            uint16_t *data = stbi_load_16(file.string().c_str(), &x, &y, &n, 1);
            
            if (!data) {
                throw std::runtime_error("Failed to load terrain image: " + file.string());
            }

            if (x != resolution || y != resolution || n != 1) {
                stbi_image_free(data);
                throw std::runtime_error("Terrain image must be 16-bit grayscale with resolution " + std::to_string(resolution) + "x" + std::to_string(resolution) + ": " + file.string());
            }

            std::transform(data,
                           data + texture_.size(),
                           texture_.begin(),
                           [](stbi_us value) {
                               return static_cast<T>(value) / static_cast<T>(std::numeric_limits<stbi_us>::max());
                           });

            stbi_image_free(data);
        }

        T ReadPos(Vec3<T> const &pos) const {

            Vec2<T> texture_pos = (OctahedralEncode(pos) + T{ 1.0 }) * T{ 0.5 } * T{ kMaxCoord };
            
            T const x = std::clamp(texture_pos.X(), T{ 0.0 }, kMaxCoord);
            T const y = std::clamp(texture_pos.Y(), T{ 0.0 }, kMaxCoord);

            size_t x_i = std::min(static_cast<size_t>(x), kStride - 2);
            size_t y_i = std::min(static_cast<size_t>(y), kStride - 2);

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
        std::array<T, resolution * resolution> texture_{};
    };
    
}

