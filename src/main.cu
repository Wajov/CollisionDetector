#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#include "Vec.h"
#include "Tri.h"
#include "Box.h"
#include "BvhNode.h"
#include "CheckHelper.h"

extern double getTime();

void detectCollisionCpuNaive(int vtxNum, Vec* vtxs, int triNum, Tri* tris) {
    double t = getTime();
    std::cout << "Detecting Collision with CPU-Naive..." << std::endl;

    int num = 0;
    for (int i = 0; i < triNum; i++)
        for (int j = i + 1; j < triNum; j++)
            if (triCollision(vtxs, tris, i, j))
                num++;

    std::cout << "Totally " << num << " colliding pairs..." << std::endl;
    std::cout << "End detecting: " << getTime() - t << " seconds" << std::endl << std::endl;
}

void buildBvhCpu(int vtxNum, Vec* vtxs, int triNum, Tri* tris, int &size, Box *&boxes, BvhNode *&bvh) {
    boxes = new Box[triNum];
    for (int i = 0; i < triNum; i++) {
        boxes[i].box.add(vtxs[tris[i].i]);
        boxes[i].box.add(vtxs[tris[i].j]);
        boxes[i].box.add(vtxs[tris[i].k]);
        boxes[i].center = (boxes[i].box.min + boxes[i].box.min) * 0.5f;
        boxes[i].index = i;
    }
    std::sort(boxes, boxes + triNum);

    for (size = 1; size < triNum; size <<= 1);
    bvh = new BvhNode[size << 1];
    for (int i = 0; i < triNum; i++) {
        bvh[size + i].box = boxes[i].box;
        bvh[size + i].maxIdx = boxes[i].index;
    }
    for (int i = size - 1; i > 0; i--) {
        bvh[i].box.combine(bvh[i << 1].box);
        bvh[i].box.combine(bvh[(i << 1) + 1].box);
        bvh[i].maxIdx = std::max(bvh[i << 1].maxIdx, bvh[(i << 1) + 1].maxIdx);
    }
}

void queryBvhCpu(Vec* vtxs, Tri* tris, int size, BvhNode *bvh, Box& box, int &num) {
    if (!box.box.overlap(bvh[1].box))
        return;

    int stack[64], top = 0, node = 1;
    stack[0] = 0;

    do {
        int childL = node << 1;
        int childR = (node << 1) + 1;
        bool overlapL = box.box.overlap(bvh[childL].box) && box.index < bvh[childL].maxIdx;
        bool overlapR = box.box.overlap(bvh[childR].box) && box.index < bvh[childR].maxIdx;

        if (overlapL && childL >= size && triCollision(vtxs, tris, box.index, bvh[childL].maxIdx))
            num++;
        if (overlapR && childR >= size && triCollision(vtxs, tris, box.index, bvh[childR].maxIdx))
            num++;

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
}

void detectCollisionCpuBvh(int vtxNum, Vec* vtxs, int triNum, Tri* tris) {
    double t = getTime();
    std::cout << "Detecting Collision with CPU-BVH..." << std::endl;

    int num = 0;
    int size;
    Box* boxes;
    BvhNode* bvh;
    buildBvhCpu(vtxNum, vtxs, triNum, tris, size, boxes, bvh);

    for (int i = 0; i < triNum; i++)
        queryBvhCpu(vtxs, tris, size, bvh, boxes[i], num);

    std::cout << "Totally " << num << " colliding pairs..." << std::endl;
    std::cout << "End detecting: " << getTime() - t << " seconds" << std::endl << std::endl;
}

extern void detectCollisionGpuBvh(int vtxNum, Vec* vtxs, int triNum, Tri* tris);

int main() {
    std::ifstream fin("flag/0108_00.obj");
    std::string s;
    std::vector<Vec> vertices;
    std::vector<Tri> triangles;

    while (fin >> s) {
        if (s == "v") {
            float x, y, z;
            fin >> x >> y >> z;
            vertices.push_back(Vec(x, y, z));
        }
        else if (s == "f") {
            int i, j, k;
            std::string s1, s2, s3;
            fin >> i >> s1 >> j >> s2 >> k >> s3;
            triangles.push_back(Tri(i - 1, j - 1, k - 1));
        }
        getline(fin, s);
    }

    int vtxNum = vertices.size();
    Vec *vtxs = new Vec[vtxNum];
    memcpy(vtxs, &vertices[0], vtxNum * sizeof(Vec));
    int triNum = triangles.size();
    Tri *tris = new Tri[triNum];
    memcpy(tris, &triangles[0], triNum * sizeof(Tri));
    std::cout << "Model loaded, " << vtxNum << " vertices, " << triNum << " triangles." << std::endl;

    int opt;
    while (std::cin >> opt) {
        switch (opt) {
        case 1:
            detectCollisionCpuNaive(vtxNum, vtxs, triNum, tris);
            break;
        case 2:
            detectCollisionCpuBvh(vtxNum, vtxs, triNum, tris);
            break;
        case 3:
            detectCollisionGpuBvh(vtxNum, vtxs, triNum, tris);
            break;
        }
    }

    return 0;
}