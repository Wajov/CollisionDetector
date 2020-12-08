#include <iostream>
#include <sys/time.h>

#include <cuda_runtime.h>
#include <thrust/sort.h>
#include <thrust/device_vector.h>

#include "Vec.h"
#include "Tri.h"
#include "AABB.h"
#include "Box.h"
#include "BvhNode.h"
#include "CheckHelper.h"

double getTime() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec + t.tv_usec / 1000000.0;
}

__global__ void calculateBox(int upper, Vec *vtxs, Tri *tris, Box *boxes) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= upper)
        return;
    Tri tri = tris[i];
    AABB box;
    box.add(vtxs[tri.i]);
    box.add(vtxs[tri.j]);
    box.add(vtxs[tri.k]);
    Vec center = (box.min + box.max) * 0.5f;
    boxes[i] = Box(box, center, i);
}

__global__ void calculateBvhLeaf(int upper, int size, Box* boxes, BvhNode* bvh) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= upper)
        return;
    Box box = boxes[i];
    bvh[size + i] = BvhNode(box.box, box.index);
}

__global__ void calculateBvhInner(int upper, int size, BvhNode *bvh) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= upper)
        return;
    i += size;
    BvhNode childL = bvh[i << 1];
    BvhNode childR = bvh[(i << 1) + 1];
    AABB box;
    box.combine(childL.box);
    box.combine(childR.box);
    int maxIdx = childL.maxIdx > childR.maxIdx ? childL.maxIdx : childR.maxIdx;
    bvh[i] = BvhNode(box, maxIdx);
}

struct Less {
    __host__ __device__ bool operator()(const Box& a, const Box& b) {
        if (a.center.x != b.center.x)
            return a.center.x < b.center.x;
        else if (a.center.y != b.center.y)
            return a.center.y < b.center.y;
        else
            return a.center.z < b.center.z;
    }
};

void buildBvhGpu(int vtxNum, Vec* vtxs, int triNum, Tri* tris, int& size, Box*& boxes, BvhNode*& bvh) {
    cudaMalloc(&boxes, triNum * sizeof(Box));
    calculateBox<<<(triNum + 1023) / 1024, 1024>>>(triNum, vtxs, tris, boxes);
    
    thrust::device_ptr<Box> boxPtr(boxes);
    thrust::sort(boxPtr, boxPtr + triNum, Less());

    for (size = 1; size < triNum; size <<= 1);
    cudaMalloc(&bvh, (size << 1) * sizeof(BvhNode));
    calculateBvhLeaf<<<(triNum + 1023) / 1024, 1024>>>(triNum, size, boxes, bvh);
    for (int levelSize = size >> 1; levelSize > 0; levelSize >>= 1)
        calculateBvhInner<<<(levelSize + 15) / 16, 16>>>(levelSize, levelSize, bvh);
}

__global__ void queryBvhGpu(int upper, Vec* vtxs, Tri* tris, int size, BvhNode* bvh, Box* boxes, int *num) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= upper)
        return;
    Box box = boxes[i];
    if (!box.box.overlap(bvh[1].box))
        return;

    int stack[64], top = 0, node = 1, numCollision = 0;
    stack[0] = 0;

    do {
        int childL = node << 1;
        int childR = (node << 1) + 1;
        BvhNode nodeL = bvh[childL];
        BvhNode nodeR = bvh[childR];
        bool overlapL = box.box.overlap(nodeL.box) && box.index < nodeL.maxIdx;
        bool overlapR = box.box.overlap(nodeR.box) && box.index < nodeR.maxIdx;

        if (overlapL && childL >= size && triCollision(vtxs, tris, box.index, nodeL.maxIdx))
            numCollision++;
        if (overlapR && childR >= size && triCollision(vtxs, tris, box.index, nodeR.maxIdx))
            numCollision++;

        bool traverseL = (overlapL && childL < size);
        bool traverseR = (overlapR && childR < size);
        if (!traverseL && !traverseR)
            node = stack[top--];
        else {
            node = traverseL ? childL : childR;
            if (traverseL && traverseR)
                stack[++top] = childR;
        }
    } while (node > 0);
    atomicAdd(num, numCollision);
}

void detectCollisionGpuBvh(int vtxNum, Vec* vtxs, int triNum, Tri* tris) {
    double t = getTime();
    std::cout << "Detecting Collision with GPU-BVH..." << std::endl;

    Vec* vtxsGpu;
    cudaMalloc(&vtxsGpu, vtxNum * sizeof(Vec));
    cudaMemcpy(vtxsGpu, vtxs, vtxNum * sizeof(Vec), cudaMemcpyHostToDevice);
    Tri* trisGpu;
    cudaMalloc(&trisGpu, triNum * sizeof(Tri));
    cudaMemcpy(trisGpu, tris, triNum * sizeof(Tri), cudaMemcpyHostToDevice);

    int *num, *numGpu;
    int size;
    Box* boxes;
    BvhNode* bvh;
    buildBvhGpu(vtxNum, vtxsGpu, triNum, trisGpu, size, boxes, bvh);
    
    cudaMalloc(&numGpu, sizeof(int));
    cudaMemset(numGpu, 0, sizeof(int));
    queryBvhGpu<<<(triNum + 255) / 256, 256>>>(triNum, vtxsGpu, trisGpu, size, bvh, boxes, numGpu);
    num = new int;
    cudaMemcpy(num, numGpu, sizeof(int), cudaMemcpyDeviceToHost);

    cudaFree(vtxsGpu);
    cudaFree(trisGpu);
    cudaFree(boxes);
    cudaFree(bvh);
    cudaFree(numGpu);

    std::cout << "Totally " << *num << " colliding pairs..." << std::endl;
    std::cout << "End detecting: " << getTime() - t << " seconds" << std::endl << std::endl;
}