#include "bvh.h"
#include "logger.h"

namespace coacd
{
    BVH::BVH(const Model _model)
    {
        this->model = _model;
        BuildBVH();
    }

    void BVH::BuildBVH()
    {
        const int N = model.triangles.size();
        bvhNode.resize(N * 2 - 1);
        rootNodeIdx = 0;
        nodesUsed = 1;

        // compute triangle centroids
        centroids.resize(N);
        for (int i = 0; i < N; i++)
        {
            const vec3i tri = model.triangles[i];
            for (int j = 0; j < 3; j++)
                centroids[i][j] = (model.points[tri[0]][j] + model.points[tri[1]][j] + model.points[tri[2]][j]) / 3.0f;
        }
        // assign all triangles to root node
        BVHNode &root = bvhNode[rootNodeIdx];
        root.left = root.right = 0;
        root.firstTri = 0, root.numTri = N;
        UpdateNodeBounds(rootNodeIdx);
        // subdivide recursively
        Subdivide(rootNodeIdx);
    }

    void BVH::UpdateNodeBounds(int nodeIdx)
    {
        BVHNode &node = bvhNode[nodeIdx];
        node.aabbMin[0] = node.aabbMin[1] = node.aabbMin[2] = 1e10;
        node.aabbMax[0] = node.aabbMax[1] = node.aabbMax[2] = -1e10;
        for (int first = node.firstTri, i = 0; i < node.numTri; i++)
        {
            const vec3i tri = model.triangles[first + i];
            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    node.aabbMin[j] = min(node.aabbMin[j], model.points[tri[k]][j]);
                    node.aabbMax[j] = max(node.aabbMax[j], model.points[tri[k]][j]);
                }
            }
        }
    }

    void BVH::Subdivide(int nodeIdx)
    {
        // terminate recursion
        BVHNode &node = bvhNode[nodeIdx];
        if (node.numTri <= 2)
        {
            return;
        }
        // determine split axis and position
        double extent[3] = {node.aabbMax[0] - node.aabbMin[0], node.aabbMax[1] - node.aabbMin[1], node.aabbMax[2] - node.aabbMin[2]};
        int axes[3];
        if (extent[1] > extent[0])
        {
            axes[0] = 1;
            axes[1] = 0;
        }
        else
        {
            axes[0] = 0;
            axes[1] = 1;
        }
        if (extent[2] > extent[axes[1]])
        {
            axes[2] = axes[1];
            if (extent[2] > extent[axes[0]])
            {
                axes[1] = axes[0];
                axes[0] = 2;
            }
            else
                axes[1] = 2;
        }
        else
            axes[2] = 2;

        // travers all the axes from widest to narrowest, in case long thin triangles problem
        for (int ax = 0; ax < 3; ax++)
        {
            int axis = axes[ax];
            double splitPos = node.aabbMin[axis] + extent[axis] * 0.5;
            // in-place partition
            int i = node.firstTri;
            int j = i + node.numTri - 1;
            while (i <= j)
            {
                if (centroids[i][axis] < splitPos)
                    i++;
                else
                {
                    std::swap(model.triangles[i], model.triangles[j]);
                    std::swap(centroids[i], centroids[j--]);
                }
            }

            // abort split if one of the sides is empty
            int leftCount = i - node.firstTri;
            if ((leftCount == 0 || leftCount == node.numTri) && ax != 2)
            {
                continue;
            }
            else if ((leftCount == 0 || leftCount == node.numTri) && ax == 2)
            {
                return;
            }

            // create child nodes
            int leftChildIdx = nodesUsed++;
            int rightChildIdx = nodesUsed++;
            bvhNode[leftChildIdx].firstTri = node.firstTri;
            bvhNode[leftChildIdx].numTri = leftCount;
            bvhNode[rightChildIdx].firstTri = i;
            bvhNode[rightChildIdx].numTri = node.numTri - leftCount;
            node.left = leftChildIdx;
            node.right = rightChildIdx;
            node.numTri = 0;
            // update bounds
            UpdateNodeBounds(leftChildIdx);
            UpdateNodeBounds(rightChildIdx);
            // recurse
            Subdivide(leftChildIdx);
            Subdivide(rightChildIdx);
            break;
        }
    }

    bool isOverlap1D(double xmin1, double xmin2, double xmax1, double xmax2)
    {
        assert(xmin1 <= xmax1 && xmin2 <= xmax2);
        return (xmin1 <= xmax2 && xmax1 >= xmin2);
    }
    bool isOverlap3D(vec3d aabbMin1, vec3d aabbMin2, vec3d aabbMax1, vec3d aabbMax2)
    {
        assert(aabbMin1[0] <= aabbMax1[0] && aabbMin1[1] <= aabbMax1[1] && aabbMin1[2] <= aabbMax1[2] && aabbMin2[0] <= aabbMax2[0] && aabbMin2[1] <= aabbMax2[1] && aabbMin2[2] <= aabbMax2[2]);
        return (isOverlap1D(aabbMin1[0], aabbMin2[0], aabbMax1[0], aabbMax2[0]) &&
                isOverlap1D(aabbMin1[1], aabbMin2[1], aabbMax1[1], aabbMax2[1]) &&
                isOverlap1D(aabbMin1[2], aabbMin2[2], aabbMax1[2], aabbMax2[2]));
    }

    bool BVH::IntersectAABB(vec3i triangleIdx, const vec3d &aabbMin, const vec3d &aabbMax)
    {
        vec3d tri_aabbMin = {1e10, 1e10, 1e10};
        vec3d tri_aabbMax = {-1e10, -1e10, -1e10};
        for (int i = 0; i < 3; i++)
        {
            tri_aabbMin[i] = min(tri_aabbMin[i], model.points[triangleIdx[i]][i]);
            tri_aabbMax[i] = max(tri_aabbMax[i], model.points[triangleIdx[i]][i]);
        }
        return isOverlap3D(tri_aabbMin, aabbMin, tri_aabbMax, aabbMax);
    }

    bool BVH::IntersectBVH(vec3i triangleIdx, const int nodeIdx)
    {
        BVHNode &node = bvhNode[nodeIdx];
        if (!IntersectAABB(triangleIdx, node.aabbMin, node.aabbMax))
            return false;
        if (node.isLeaf())
        {
            for (int i = 0; i < node.numTri; i++)
            {
                IntersectVector3 v0, v1, v2, u0, u1, u2;
                const vec3i t0 = triangleIdx;
                const vec3i t1 = model.triangles[node.firstTri + i];
                const vec3d p0_0 = model.points[t0[0]];
                const vec3d p0_1 = model.points[t0[1]];
                const vec3d p0_2 = model.points[t0[2]];
                const vec3d p1_0 = model.points[t1[0]];
                const vec3d p1_1 = model.points[t1[1]];
                const vec3d p1_2 = model.points[t1[2]];

                v0 = {float(p0_0[0]), float(p0_0[1]), float(p0_0[2])};
                v1 = {float(p0_1[0]), float(p0_1[1]), float(p0_1[2])};
                v2 = {float(p0_2[0]), float(p0_2[1]), float(p0_2[2])};
                u0 = {float(p1_0[0]), float(p1_0[1]), float(p1_0[2])};
                u1 = {float(p1_1[0]), float(p1_1[1]), float(p1_1[2])};
                u2 = {float(p1_2[0]), float(p1_2[1]), float(p1_2[2])};

                if (t0[0] != t1[0] && t0[0] != t1[1] && t0[0] != t1[2] &&
                    t0[1] != t1[0] && t0[1] != t1[1] && t0[1] != t1[2] &&
                    t0[2] != t1[0] && t0[2] != t1[1] && t0[2] != t1[2] &&
                    !SamePointDetect(p0_0, p1_0) && !SamePointDetect(p0_0, p1_1) && !SamePointDetect(p0_0, p1_2) &&
                    !SamePointDetect(p0_1, p1_0) && !SamePointDetect(p0_1, p1_1) && !SamePointDetect(p0_1, p1_2) &&
                    !SamePointDetect(p0_2, p1_0) && !SamePointDetect(p0_2, p1_1) && !SamePointDetect(p0_2, p1_2))
                {
                    bool flag = threeyd::moeller::TriangleIntersects<IntersectVector3>::triangle(v0, v1, v2, u0, u1, u2);
                    if (flag)
                    {
                        // std::cout << "v " << v0[0] << ' ' << v0[1] << ' ' << v0[2] << std::endl;
                        // std::cout << "v " << v1[0] << ' ' << v1[1] << ' ' << v1[2] << std::endl;
                        // std::cout << "v " << v2[0] << ' ' << v2[1] << ' ' << v2[2] << std::endl;
                        // std::cout << "v " << u0[0] << ' ' << u0[1] << ' ' << u0[2] << std::endl;
                        // std::cout << "v " << u1[0] << ' ' << u1[1] << ' ' << u1[2] << std::endl;
                        // std::cout << "v " << u2[0] << ' ' << u2[1] << ' ' << u2[2] << std::endl;
                        // std::cout << "f 1 2 3" << std::endl;
                        // std::cout << "f 4 5 6" << std::endl;
                        return true;
                    }
                }
            }
            return false;
        }
        else
        {
            if (node.left != -1)
            {
                bool left_intersect = IntersectBVH(triangleIdx, node.left);
                if (left_intersect)
                    return true;
            }
            if (node.right != -1)
            {
                bool right_intersect = IntersectBVH(triangleIdx, node.right);
                return right_intersect;
            }
            return false;
        }
    }
}
