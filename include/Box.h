#ifndef BOX_H
#define BOX_H

#include "AABB.h"
#include "Vec.h"

#include <cuda_runtime.h>

class Box {
public:
    AABB box;
    Vec center;
    int index;
    __host__ __device__ Box();
    __host__ __device__ Box(AABB& box, Vec& center, int index);
    __host__ __device__ bool operator <(Box& box);
};

#endif