#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

namespace td {
    
    template<typename T>
    class TripleBuffer {
    private:
        static constexpr uint8_t kIndexMask = 0b011;
        static constexpr uint8_t kDirtyBit = 0b100;

    public:
        // Call only from UI thread
        void Publish(T const &value) noexcept {
            buffers_[back_] = value;

            uint8_t const previous = middle_.exchange(back_ | kDirtyBit, std::memory_order_acq_rel);

            back_ = previous & kIndexMask;
        }

        // Call only from Audio thread
        std::optional<T> Consume() noexcept {
            if ((middle_.load(std::memory_order_acquire) & kDirtyBit) == 0) {
                return std::nullopt;
            }

            uint8_t const previous = middle_.exchange(front_, std::memory_order_acq_rel);

            front_ = previous & kIndexMask;
            return { buffers_[front_] };
        }


    private:
        std::array<T, 3> buffers_;

        uint8_t back_{ 0 };
        std::atomic_uint8_t middle_{ 1 };
        uint8_t front_{ 2 };
    };

}
