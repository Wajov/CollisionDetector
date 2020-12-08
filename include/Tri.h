#ifndef TRI_H
#define TRI_H

#include <cuda_runtime.h>

class Tri {
public:
	int i, j, k;
	__host__ __device__ Tri();
	__host__ __device__ Tri(int i, int j, int k);
};

#endif