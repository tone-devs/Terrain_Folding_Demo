#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

#include "globals.hpp"

namespace td {
    
    template<typename T, size_t size>
    class TripleBufferBank {
    private:
        static uint8_t constexpr kIndexMask = 0b011;
        static uint8_t constexpr kDirtyBit = 0b100;

        struct alignas(kCacheLine) Slot {
            T value{};
        };

    public:
        TripleBufferBank() noexcept {
            back_.fill(0);
            for (auto &middle : middle_) {
                middle.store(1, std::memory_order_relaxed);
            }
            front_.fill(2);
        }

        // Call only from UI thread
        void Publish(size_t const slot, T const &value) noexcept {
            auto &back = back_[slot];
            buffers_[back][slot].value = value;

            uint8_t const previous = middle_[slot].exchange(back | kDirtyBit, std::memory_order_acq_rel);

            back = previous & kIndexMask;
        }

        // Call only from Audio thread
        std::optional<T> Consume(size_t const slot) noexcept {
            if ((middle_[slot].load(std::memory_order_acquire) & kDirtyBit) == 0) {
                return std::nullopt;
            }

            auto &front = front_[slot];

            uint8_t const previous = middle_[slot].exchange(front, std::memory_order_acq_rel);

            front = previous & kIndexMask;
            return { buffers_[front][slot].value };
        }


    private:
        std::array<std::array<Slot, size>, 3> buffers_;

        alignas(kCacheLine) std::array<uint8_t, size> back_;
        alignas(kCacheLine) std::array<std::atomic_uint8_t, size> middle_;
        alignas(kCacheLine) std::array<uint8_t, size> front_;
    };

}
