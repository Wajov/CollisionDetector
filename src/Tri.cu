#include "Tri.h"

__host__ __device__ Tri::Tri() {
	i = j = k = 0;
}

__host__ __device__ Tri::Tri(int i, int j, int k) {
	this->i = i;
	this->j = j;
	this->k = k;
}