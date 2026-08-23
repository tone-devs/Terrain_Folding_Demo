#pragma once

#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>

namespace td {

    template<typename T>
    class Exchanger {
    private:
        static uint8_t constexpr kIndexMask = 0b11;
        static uint8_t constexpr kDirtyBit = 0b100;

    public:
        explicit Exchanger(std::unique_ptr<T> initial)
            : slots_{
                std::move(initial),
                nullptr,
                nullptr
            } {
            assert(slots_[0]);
        }

        Exchanger(Exchanger const &) = delete;
        Exchanger &operator=(Exchanger const &) = delete;

        // CONTROL THREAD ONLY
        void Publish(std::unique_ptr<T> resource) {
            assert(resource);

            slots_[back_] = std::move(resource);
            auto const previous = middle_.exchange(back_ | kDirtyBit, std::memory_order_acq_rel);
            back_ = previous & kIndexMask;
            slots_[back_].reset();
        }

        // AUDIO THREAD ONLY
        bool Consume() noexcept {
            if ((middle_.load(std::memory_order_acquire) & kDirtyBit) == 0) {
                return false;
            }

            uint8_t const previous = middle_.exchange(front_, std::memory_order_acq_rel);

            front_ = previous & kIndexMask;

            return true;
        }

        // AUDIO THREAD ONLY
        [[nodiscard]] T const &Current() const noexcept {
            assert(slots_[front_]);
            return *slots_[front_];
        }

    private:
        std::array<std::unique_ptr<T>, 3> slots_;

        uint8_t front_{ 0 };
        std::atomic_uint8_t middle_{ 1 };
        uint8_t back_{ 2 };
    };

}