#include "BvhNode.h"

__host__ __device__ BvhNode::BvhNode() {
	maxIdx = 0;
}

__host__ __device__ BvhNode::BvhNode(AABB& box, int maxIdx) {
	this->box = box;
	this->maxIdx = maxIdx;
}