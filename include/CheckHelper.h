#ifndef CHECK_HELPER_H
#define CHECK_HELPER_H

#include <cuda_runtime.h>

#include "Vec.h"
#include "Tri.h"

__host__ __device__ inline float fmin(float a, float b, float c)
{
    float t = a;
    if (b < t)
        t = b;
    if (c < t)
        t = c;
    return t;
}

__host__ __device__ inline float fmax(float a, float b, float c)
{
    float t = a;
    if (b > t)
        t = b;
    if (c > t)
        t = c;
    return t;
}

__host__ __device__ inline bool project3(Vec& ax, Vec& p1, Vec& p2, Vec& p3)
{
    float P1 = ax.dot(p1);
    float P2 = ax.dot(p2);
    float P3 = ax.dot(p3);

    float mx1 = fmax(P1, P2, P3);
    float mn1 = fmin(P1, P2, P3);

    if (mn1 > 0)
        return false;
    if (0 > mx1)
        return false;
    return true;
}

__host__ __device__ inline bool project6(Vec& ax, Vec& p1, Vec& p2, Vec& p3, Vec& q1, Vec& q2, Vec& q3)
{
    float P1 = ax.dot(p1);
    float P2 = ax.dot(p2);
    float P3 = ax.dot(p3);
    float Q1 = ax.dot(q1);
    float Q2 = ax.dot(q2);
    float Q3 = ax.dot(q3);

    float mx1 = fmax(P1, P2, P3);
    float mn1 = fmin(P1, P2, P3);
    float mx2 = fmax(Q1, Q2, Q3);
    float mn2 = fmin(Q1, Q2, Q3);

    if (mn1 > mx2)
        return false;
    if (mn2 > mx1)
        return false;
    return true;
}

__host__ __device__ inline bool triContact(Vec& P1, Vec& P2, Vec& P3, Vec& Q1, Vec& Q2, Vec& Q3)
{
    Vec p1;
    Vec p2 = P2 - P1;
    Vec p3 = P3 - P1;
    Vec q1 = Q1 - P1;
    Vec q2 = Q2 - P1;
    Vec q3 = Q3 - P1;

    Vec e1 = p2 - p1;
    Vec e2 = p3 - p2;
    Vec e3 = p1 - p3;

    Vec f1 = q2 - q1;
    Vec f2 = q3 - q2;
    Vec f3 = q1 - q3;

    Vec n1 = e1.cross(e2);
    Vec m1 = f1.cross(f2);

    Vec g1 = e1.cross(n1);
    Vec g2 = e2.cross(n1);
    Vec g3 = e3.cross(n1);

    Vec h1 = f1.cross(m1);
    Vec h2 = f2.cross(m1);
    Vec h3 = f3.cross(m1);

    Vec ef11 = e1.cross(f1);
    Vec ef12 = e1.cross(f2);
    Vec ef13 = e1.cross(f3);
    Vec ef21 = e2.cross(f1);
    Vec ef22 = e2.cross(f2);
    Vec ef23 = e2.cross(f3);
    Vec ef31 = e3.cross(f1);
    Vec ef32 = e3.cross(f2);
    Vec ef33 = e3.cross(f3);

    if (!project3(n1, q1, q2, q3))
        return false;
    Vec t1 = Vec() - q1;
    Vec t2 = p2 - q1;
    Vec t3 = p3 - q1;
    if (!project3(m1, t1, t2, t3))
        return false;

    if (!project6(ef11, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef12, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef13, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef21, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef22, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef23, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef31, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef32, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(ef33, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(g1, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(g2, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(g3, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(h1, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(h2, p1, p2, p3, q1, q2, q3))
        return false;
    if (!project6(h3, p1, p2, p3, q1, q2, q3))
        return false;

    return true;
}

__host__ __device__ inline bool triCollision(Vec* vtxs, Tri* tris, int i, int j) {
    Tri a = tris[i];
    Tri b = tris[j];

    if (a.i == b.i || a.i == b.j || a.i == b.k)
        return false;
    if (a.j == b.i || a.j == b.j || a.j == b.k)
        return false;
    if (a.k == b.i || a.k == b.j || a.k == b.k)
        return false;

    Vec p0 = vtxs[a.i];
    Vec p1 = vtxs[a.j];
    Vec p2 = vtxs[a.k];
    Vec q0 = vtxs[b.i];
    Vec q1 = vtxs[b.j];
    Vec q2 = vtxs[b.k];

    return triContact(p0, p1, p2, q0, q1, q2);
}

#endif