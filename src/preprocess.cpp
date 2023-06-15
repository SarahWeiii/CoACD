#include "preprocess.h"
#include "logger.h"
#include "shape.h"
// #include "intersection.h"

namespace coacd
{
    void SDFManifold(Model &input, Model &output, double scale, double level_set)
    {
        std::vector<Vec3s> points;
        std::vector<Vec3I> tris;
        std::vector<Vec4I> quads;

        logger::info(" - Preprocess");
        logger::info("Preprocess resolution: {}", scale);

        clock_t start, end;
        start = clock();

        for (unsigned int i = 0; i < input.points.size(); ++i)
        {
            points.push_back({input.points[i][0] * scale, input.points[i][1] * scale, input.points[i][2] * scale});
        }
        for (unsigned int i = 0; i < input.triangles.size(); ++i)
        {
            tris.push_back({(unsigned int)input.triangles[i][0], (unsigned int)input.triangles[i][1], (unsigned int)input.triangles[i][2]});
        }

        math::Transform::Ptr xform = math::Transform::createLinearTransform();
        tools::QuadAndTriangleDataAdapter<Vec3s, Vec3I> mesh(points, tris);

        DoubleGrid::Ptr sgrid = tools::meshToSignedDistanceField<DoubleGrid>(
            *xform, points, tris, quads, 3.0, 3.0);

        std::vector<Vec3s> newPoints;
        std::vector<Vec3I> newTriangles;
        std::vector<Vec4I> newQuads;
        tools::volumeToMesh(*sgrid, newPoints, newTriangles, newQuads, 0.55);

        output.Clear();
        for (unsigned int i = 0; i < newPoints.size(); ++i)
        {
            output.points.push_back({newPoints[i][0] / scale, newPoints[i][1] / scale, newPoints[i][2] / scale});
        }
        for (unsigned int i = 0; i < newTriangles.size(); ++i)
        {
            output.triangles.push_back({(int)newTriangles[i][0], (int)newTriangles[i][2], (int)newTriangles[i][1]});
        }
        for (unsigned int i = 0; i < newQuads.size(); ++i)
        {
            output.triangles.push_back({(int)newQuads[i][0], (int)newQuads[i][2], (int)newQuads[i][1]});
            output.triangles.push_back({(int)newQuads[i][0], (int)newQuads[i][3], (int)newQuads[i][2]});
        }
        end = clock();

        logger::info("Preprocess Time: {}s", double(end - start) / CLOCKS_PER_SEC);
    }

    bool IsManifold(Model &input)
    {
        logger::info(" - Manifold Check");
        clock_t start, end;
        start = clock();
        // Check all edges are shared by exactly two triangles (watertight manifold)
        vector<pair<int, int>> edges;
        map<pair<int, int>, int> edge_num;
        for (int i = 0; i < (int)input.triangles.size(); i++)
        {
            int idx0 = input.triangles[i][0];
            int idx1 = input.triangles[i][1];
            int idx2 = input.triangles[i][2];
            edges.push_back({idx0, idx1});
            edges.push_back({idx1, idx2});
            edges.push_back({idx2, idx0});

            if (!edge_num.contains({idx0, idx1}))
                edge_num[{idx0, idx1}] = 1;
            else
                return false;
            if (!edge_num.contains({idx1, idx2}))
                edge_num[{idx1, idx2}] = 1;
            else
                return false;
            if (!edge_num.contains({idx2, idx0}))
                edge_num[{idx2, idx0}] = 1;
            else
                return false;
        }

        for (int i = 0; i < (int)edges.size(); i++)
        {
            pair<int, int> oppo_edge = {edges[i].second, edges[i].first};
            if (!edge_num.contains(oppo_edge))
                return false;
        }
        logger::info("[1/3] Edge check finish");

        // Check self-intersection
        BVH bvhTree(input);
        for (int i = 0; i < (int)input.triangles.size(); i++)
        {
            bool is_intersect = bvhTree.IntersectBVH(input.triangles[i], 0);
            if (is_intersect)
            {
                return false;
            }
        }
        logger::info("[2/3] Self-intersection check finish");

        // Check triange orientation
        double mesh_vol = MeshVolume(input);
        if (mesh_vol < 0)
        {
            // Reverse all the triangles
            for (int i = 0; i < (int)input.triangles.size(); i++)
                std::swap(input.triangles[i][0], input.triangles[i][1]);
        }
        end = clock();

        logger::info("[3/3] Triangle orientation check finish. Reversed: {}", mesh_vol < 0);
        logger::info("Manifold Check Time: {}s", double(end - start) / CLOCKS_PER_SEC);

        return true;
    }

}