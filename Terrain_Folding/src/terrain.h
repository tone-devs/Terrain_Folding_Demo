#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

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
        static size_t constexpr kStride = resolution + 1;
        static T constexpr kMaxCoord = T{ resolution };

    public:
        explicit Terrain(std::filesystem::path const &file) {
            terrain_ = std::make_unique_for_overwrite<std::array<T, kStride * kStride * 4>>();
            LoadTexture(file);
        }

        void LoadTexture(std::filesystem::path const &file) {
            int width, height, n;

            std::string const file_name = file.string();

            if (!stbi_is_16_bit(file_name.c_str())) {
                throw std::runtime_error("Terrain image must be 16-bit: " + file.string());
            }

            uint16_t *data = stbi_load_16(file.string().c_str(), &width, &height, &n, 1);
            
            if (!data) {
                throw std::runtime_error("Failed to load terrain image: " + file.string());
            }

            if (static_cast<size_t>(width) < 2 * resolution || static_cast<size_t>(height) < resolution || n != 1) {
                stbi_image_free(data);
                throw std::runtime_error("Terrain image must be 16-bit grayscale with resolution at least" + std::to_string(2 * resolution) + "x" + std::to_string(resolution) + ": " + file.string());
            }

            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < kStride; ++j) {
                    for (size_t k = 0; k < kStride; ++k) {
                        Vec2<T> const uv = Vec2<T>{ static_cast<T>(k), static_cast<T>(j) } / static_cast<T>(kMaxCoord);
                        
                        bool const z_positive = uv.L1() <= T{ 1.0 };
                        Vec3<T> const unsigned_pos{ z_positive ? uv : (T{ 1.0 } - uv).YX(), std::abs(T{ 1.0 } - uv.L1()) };
                        Vec3<T> const signs = Vec3<T>{ static_cast<T>(i / 2), static_cast<T>(i % 2), static_cast<T>(z_positive) } * T{ 2.0 } - T{ 1.0 };
                        Vec3<T> pos = unsigned_pos.HadamardProd(signs);
                        pos = pos.Norm();
                        
                        // map form R^3 to Equirectangular projection
                        T const texture_x = static_cast<T>(width) * (atan2(pos.Y(), pos.X()) * T{ 0.5 } * std::numbers::inv_pi_v<T> + T{ 0.5 });
                        T const texture_y = static_cast<T>(height) * acos(std::clamp(pos.Z(), T{ -1.0 }, T{ 1.0 })) * std::numbers::inv_pi_v<T>;

                        T const x = std::fmod(texture_x, static_cast<T>(width));
                        T const y = std::clamp(texture_y, T{ 0.0 }, static_cast<T>(height - 1));

                        size_t const x_0 = static_cast<size_t>(x);
                        size_t const x_1 = (x_0 + 1) % static_cast<size_t>(width);

                        size_t const y_0 = std::min(static_cast<size_t>(y), static_cast<size_t>(height - 2));
                        size_t const y_1 = y_0 + 1;

                        T const texture_x_f = x - static_cast<T>(x_0);
                        T const texture_y_f = y - static_cast<T>(y_0);

                        auto scale = [](uint16_t const value) -> float {
                                return 2.0f * static_cast<float>(value) / static_cast<float>(std::numeric_limits<stbi_us>::max()) - 1.0f;
                            };

                        float const p_0_0 = scale(data[y_0 * static_cast<size_t>(width) + x_0]);
                        float const p_0_1 = scale(data[y_0 * static_cast<size_t>(width) + x_1]);
                        float const p_1_0 = scale(data[y_1 * static_cast<size_t>(width) + x_0]);
                        float const p_1_1 = scale(data[y_1 * static_cast<size_t>(width) + x_1]);

                        float const p_0_x = p_0_0 * (1.0f - texture_x_f) + p_0_1 * texture_x_f;
                        float const p_1_x = p_1_0 * (1.0f - texture_x_f) + p_1_1 * texture_x_f;

                        (*terrain_)[(i * kStride + j) * kStride + k] = static_cast<T>(p_0_x * (1.0f - texture_y_f) + p_1_x * texture_y_f);
                    }
                }
            }

            stbi_image_free(data);
        }

        template<typename TP>
            requires (std::floating_point<TP>)
        T ReadPos(Vec3<TP> const &pos) const {
            Vec2<TP> const patch_index_vec = (pos.XY().Sign() + TP{ 1.0 }) * TP{ 0.5 };
            size_t const patch_index = static_cast<size_t>(patch_index_vec.X() * TP{ 2.0 }) + static_cast<size_t>(patch_index_vec.Y());

            Vec2<TP> const uv = pos.Z() >= TP{ 0.0 } ? (pos.Abs() / pos.L1()).XY() : TP{ 1.0 } - (pos.Abs() / pos.L1()).YX();
            Vec2<TP> const tex_coord = uv * kMaxCoord;

            size_t const x_i = static_cast<size_t>(std::min(tex_coord.X(), kMaxCoord - TP{ 1.0 }));
            size_t const y_i = static_cast<size_t>(std::min(tex_coord.Y(), kMaxCoord - TP{ 1.0 }));

            T const x_f = static_cast<T>(tex_coord.X() - static_cast<TP>(x_i));
            T const y_f = static_cast<T>(tex_coord.Y() - static_cast<TP>(y_i));

            bool const upper_tri = (x_f + y_f) > T{ 1.0 };

            T result;
            auto &terrain = *terrain_;
            if (upper_tri) {
                T const v_1 = terrain[patch_index * kStride * kStride + (y_i + 1) * kStride + (x_i + 1)];
                T const v_2 = terrain[patch_index * kStride * kStride + (y_i + 1) * kStride + x_i];
                T const v_3 = terrain[patch_index * kStride * kStride + y_i * kStride + (x_i + 1)];

                result = v_1 * (x_f + y_f - T{ 1.0 }) +
                         v_2 * (T{ 1.0 } - x_f) +
                         v_3 * (T{ 1.0 } - y_f);
            } else {
                T const v_1 = terrain[patch_index * kStride * kStride + y_i * kStride + x_i];
                T const v_2 = terrain[patch_index * kStride * kStride + y_i * kStride + (x_i + 1)];
                T const v_3 = terrain[patch_index * kStride * kStride + (y_i + 1) * kStride + x_i];

                result = v_1 * (T{ 1.0 } - x_f - y_f) +
                         v_2 * x_f +
                         v_3 * y_f;
            }

            return result;
        }

    private:
        std::unique_ptr<std::array<T, kStride * kStride * 4>> terrain_;
    };
    
}

