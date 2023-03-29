#pragma once

#include "model_obj.h"
#include <deque>

namespace coacd
{
    void SimpleCyclesFromEdges(const std::vector<std::pair<int, int>> edges, std::vector<std::vector<int>> &simple_cycles);
    void FindCycleDirection(std::vector<vec3d> border, std::vector<std::vector<int>> cycles, Plane plane, std::map<std::pair<int, int>, bool> &cycles_dir);
    void RemoveOutlierTriangles(std::vector<vec3d> border, std::vector<vec3d> overlap, std::vector<std::pair<int, int>> border_edges, int oriN,
                                std::vector<vec3i> border_triangles, std::vector<vec3i> &final_triangles);
    void RecordIntersectPoint(Model mesh, std::map<std::pair<int, int>, int> &edge_map, int i, int ep0, int ep1, int &idx, std::vector<vec3d> &border, vec3d point);
    bool Clip(const Model &mesh, Model &pos, Model &neg, Plane &plane, double &cut_area, bool foo = false);
    bool CreatePlaneRotationMatrix(std::vector<vec3d> &border, std::vector<std::pair<int, int>> border_edges, vec3d &T, double R[3][3], Plane &plane);
    short Triangulation(std::vector<vec3d> &border, std::vector<std::pair<int, int>> border_edges, std::vector<vec3i> &border_triangles, Plane &plane);
    void PrintEdgeSet(std::vector<std::pair<int, int>> edges);

    inline void addPoint(std::map<int, int> &vertex_map, std::vector<vec3d> &border, vec3d pt, int id, int &idx)
    {
        if (vertex_map.find(id) == vertex_map.end())
        {
            int flag = -1;
            for (int i = 0; i < (int)border.size(); i++)
            {
                if ((fabs(border[i][0] - pt[0])) < 1e-4 && (fabs(border[i][1] - pt[1])) < 1e-4 && (fabs(border[i][2] - pt[2])) < 1e-4)
                {
                    flag = i;
                    break;
                }
            }
            if (flag == -1)
            {
                vertex_map[id] = idx;
                border.push_back(pt);
                idx++;
            }
            else
                vertex_map[id] = flag;
        }
    }

    inline void addEdgePoint(std::map<std::pair<int, int>, int> &edge_map, std::vector<vec3d>& border, vec3d pt, int id1, int id2, int& idx)
    {
        std::pair<int, int> edge1 = std::make_pair(id1, id2);
        std::pair<int, int> edge2 = std::make_pair(id2, id1);
        if (edge_map.find(edge1) == edge_map.end() && edge_map.find(edge2) == edge_map.end())
        {
            int flag = -1;
            for (int i = 0; i < (int)border.size(); i++)
            {
                if ((fabs(border[i][0] - pt[0])) < 1e-4 && (fabs(border[i][1] - pt[1])) < 1e-4 && (fabs(border[i][2] - pt[2])) < 1e-4)
                {
                    flag = i;
                    break;
                }
            }
            if (flag == -1)
            {
                edge_map[edge1] = idx;
                edge_map[edge2] = idx;
                border.push_back(pt);
                idx++;
            }
            else
            {
                edge_map[edge1] = flag;
                edge_map[edge2] = flag;
            }
        }
    }

    inline bool FaceOverlap(std::map<int, bool> overlap_map, vec3i triangle)
    {
        int idx0 = triangle[0], idx1 = triangle[1], idx2 = triangle[2];
        if (overlap_map.find(idx0) == overlap_map.end() && overlap_map.find(idx1) == overlap_map.end() &&
            overlap_map.find(idx2) == overlap_map.end())
            return false;
        return true;
    }
}