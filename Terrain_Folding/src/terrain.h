#pragma once

#include <concepts>

#include"vec.h"

namespace td {

    template<typename T>
        requires (std::floating_point<T>)
    class Terrain 
    {
    public:
        T ReadPos(Vec3<T> const &pos) {
            // Todo: implement
        }

    private:
        size_t CalculateFaceOffset(Vec3<T> const &pos) {
            
        }
    };
    
}

