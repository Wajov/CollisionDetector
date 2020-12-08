#ifndef BVH_NODE_H
#define BVH_NODE_H

#include "AABB.h"

#include <cuda_runtime.h>

class BvhNode {
public:
    AABB box;
    int maxIdx;
    __host__ __device__ BvhNode();
    __host__ __device__ BvhNode(AABB &box, int maxIdx);
};

#endif