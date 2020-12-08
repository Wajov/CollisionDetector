#include "Vec.h"

__host__ __device__ Vec::Vec() {
	x = y = z = 0;
}

__host__ __device__ Vec::Vec(float x, float y, float z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

__host__ __device__ Vec Vec::min(Vec& vec) {
	Vec ans;
	ans.x = x < vec.x ? x : vec.x;
	ans.y = y < vec.y ? y : vec.y;
	ans.z = z < vec.z ? z : vec.z;
	return ans;
}

__host__ __device__ Vec Vec::max(Vec& vec) {
	Vec ans;
	ans.x = x > vec.x ? x : vec.x;
	ans.y = y > vec.y ? y : vec.y;
	ans.z = z > vec.z ? z : vec.z;
	return ans;
}

__host__ __device__ float Vec::dot(Vec& vec) {
	return x * vec.x + y * vec.y + z * vec.z;
}

__host__ __device__ Vec Vec::cross(Vec& vec) {
	return Vec(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x);
}

__host__ __device__ Vec Vec::operator +(Vec& vec) {
	return Vec(x + vec.x, y + vec.y, z + vec.z);
}

__host__ __device__ Vec Vec::operator -(Vec& vec) {
	return Vec(x - vec.x, y - vec.y, z - vec.z);
}

__host__ __device__ Vec Vec::operator *(float scale) {
	return Vec(x * scale, y * scale, z * scale);
}