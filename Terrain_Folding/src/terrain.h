#pragma once

#include <concepts>

namespace td {

    template<typename T>
        requires (std::floating_point<T>)
    class Terrain 
    {
    public:
        T ReadPos(Vec3<T>pos) {
            // Todo: implement
        }
    };
    
}

