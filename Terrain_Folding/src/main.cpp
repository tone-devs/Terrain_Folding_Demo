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

int main() {
    
}
