#include "AABB.h"

__host__ __device__ AABB::AABB() {
	min = Vec(FLT_MAX, FLT_MAX, FLT_MAX);
	max = Vec(-FLT_MAX, -FLT_MAX, -FLT_MAX);
}

__host__ __device__ void AABB::add(Vec& vec) {
	min = min.min(vec);
	max = max.max(vec);
}

__host__ __device__ void AABB::combine(AABB& aabb) {
	min = min.min(aabb.min);
	max = max.max(aabb.max);
}

__host__ __device__ bool AABB::overlap(AABB& aabb) {
	if (min.x > aabb.max.x)
		return false;
	if (min.y > aabb.max.y)
		return false;
	if (min.z > aabb.max.z)
		return false;

	if (max.x < aabb.min.x)
		return false;
	if (max.y < aabb.min.y)
		return false;
	if (max.z < aabb.min.z)
		return false;

	return true;
}