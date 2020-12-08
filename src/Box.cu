#include "Box.h"

__host__ __device__ Box::Box() {
	index = 0;
}

__host__ __device__ Box::Box(AABB& box, Vec& center, int index) {
	this->box = box;
	this->center = center;
	this->index = index;
}

__host__ __device__ bool Box::operator <(Box& box) {
	if (center.x != box.center.x)
		return center.x < box.center.x;
	else if (center.y != box.center.y)
		return center.y < box.center.y;
	else
		return center.z < box.center.z;
}