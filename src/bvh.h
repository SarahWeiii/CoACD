#pragma once
#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <assert.h>
#include <algorithm>
#include <set>
#include <map>
#include <unordered_map>

#include "shape.h"
#include "model_obj.h"
#include "intersection.h"

namespace coacd
{

    struct IntersectVector3
    {
        float x;
        float y;
        float z;

        float &operator[](size_t i) noexcept { return *(&x + i); }
        const float &operator[](size_t i) const noexcept { return *(&x + i); }
    };

    class BVHNode
    {
    public:
        vec3d aabbMin, aabbMax;
        int left, right, firstTri, numTri;
        bool isLeaf() { return numTri > 0; };
    };

    class BVH
    {
    public:
        Model model;
        vector<BVHNode> bvhNode;
        vector<vec3d> centroids;
        int rootNodeIdx, nodesUsed;

        BVH(const Model _model);
        void BuildBVH();
        void UpdateNodeBounds(int nodeIdx);
        void Subdivide(int nodeIdx);

        bool IntersectAABB(vec3i triangleIdx, const vec3d &aabbMin, const vec3d &aabbMax);
        bool IntersectBVH(vec3i triangleIdx, const int nodeIdx);
    };
}
