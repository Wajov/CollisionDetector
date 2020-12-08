#ifndef VEC_H
#define VEC_H

#include <cuda_runtime.h>

class Vec {
public:
	float x, y, z;
	__host__ __device__ Vec();
	__host__ __device__ Vec(float x, float y, float z);
	__host__ __device__ Vec min(Vec& vec);
	__host__ __device__ Vec max(Vec& vec);
	__host__ __device__ float dot(Vec& vec);
	__host__ __device__ Vec cross(Vec& vec);
	__host__ __device__ Vec operator +(Vec& vec);
	__host__ __device__ Vec operator -(Vec& vec);
	__host__ __device__ Vec operator *(float scale);
};

#endif