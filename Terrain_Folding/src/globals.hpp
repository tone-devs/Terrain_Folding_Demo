#pragma once

#include <limits>

namespace td {
    
    static size_t constexpr kCacheLine = 64;

    static double constexpr kSampleRate = 48'000.0;
    static size_t constexpr kBlockSize = 256;

    template<typename T>
    static T constexpr kMinAngle = T{ 1e-6 };

    template<typename T>
    static T constexpr kParallelEps = kMinAngle<T> * kMinAngle<T>;

    template<typename T>
    static T constexpr kOrthEps = T{ 64.0 } * std::numeric_limits<T>::epsilon();

    template<typename T>
    static T constexpr kR2Eps = T{ 64.0 } * std::numeric_limits<T>::epsilon();

    template<typename T>
    static T constexpr kWrapEps = T{ 16.0 } * std::numeric_limits<T>::epsilon();
    
}
