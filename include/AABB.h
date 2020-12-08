#ifndef AABB_H
#define AABB_H

#include <cfloat>

#include <cuda_runtime.h>

#include "Vec.h"

class AABB {
public:
	Vec min, max;
	__host__ __device__ AABB();
	__host__ __device__ void add(Vec &vec);
	__host__ __device__ void combine(AABB& aabb);
	__host__ __device__ bool overlap(AABB& aabb);
};

#endif

