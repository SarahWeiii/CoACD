#pragma once
#include <typeinfo>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <thread>
#include <assert.h>

#include <map>
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>

#include "nanoflann.hpp"
#include "shape.h"
using namespace std;
using namespace nanoflann;

#define INF numeric_limits<double>::max()

namespace coacd
{

    double dist_point2point(vec3d pt, vec3d p)
    {
        return sqrt(pow(pt[0] - p[0], 2) + pow(pt[1] - p[1], 2) + pow(pt[2] - p[2], 2));
    }

    double dist_point2segment(vec3d pt, vec3d s0, vec3d s1, bool flag = false)
    {
        // first we build a 3d triangle the compute the height, pt as the top point
        vec3d BA, BC;
        BA[0] = pt[0] - s1[0];
        BA[1] = pt[1] - s1[1];
        BA[2] = pt[2] - s1[2];
        BC[0] = s0[0] - s1[0];
        BC[1] = s0[1] - s1[1];
        BC[2] = s0[2] - s1[2];

        // we calculate the projected vector along the segment
        double proj_dist = (BA[0] * BC[0] + BA[1] * BC[1] + BA[2] * BC[2]) / (sqrt(pow(BC[0], 2) + pow(BC[1], 2) + pow(BC[2], 2)));
        if (flag)
        {
            vec3d proj_pt;
            double len_BC = sqrt(pow(BC[0], 2) + pow(BC[1], 2) + pow(BC[2], 2));
            proj_pt[0] = s1[0] + proj_dist / len_BC * BC[0];
            proj_pt[1] = s1[1] + proj_dist / len_BC * BC[1];
            proj_pt[2] = s1[2] + proj_dist / len_BC * BC[2];
            cout << "v " << proj_pt[0] << ' ' << proj_pt[1] << ' ' << proj_pt[2] << endl;
        }

        // we should make sure the projected point is within the segment, otherwise not consider it
        // if projected distance is negative or bigger than BC, it is out
        double valAB = sqrt(pow(BA[0], 2) + pow(BA[1], 2) + pow(BA[2], 2));
        double valBC = sqrt(pow(BC[0], 2) + pow(BC[1], 2) + pow(BC[2], 2));
        if (proj_dist < 0 || proj_dist > valBC)
            return INF;
        return sqrt(pow(valAB, 2) - pow(proj_dist, 2));
    }

    double dist_point2triangle(vec3d pt, vec3d tri_pt0, vec3d tri_pt1, vec3d tri_pt2, bool flag = false)
    {
        // calculate the funciton of the plane, n = (a, b, c)
        double _a = (tri_pt1[1] - tri_pt0[1]) * (tri_pt2[2] - tri_pt0[2]) - (tri_pt1[2] - tri_pt0[2]) * (tri_pt2[1] - tri_pt0[1]);
        double _b = (tri_pt1[2] - tri_pt0[2]) * (tri_pt2[0] - tri_pt0[0]) - (tri_pt1[0] - tri_pt0[0]) * (tri_pt2[2] - tri_pt0[2]);
        double _c = (tri_pt1[0] - tri_pt0[0]) * (tri_pt2[1] - tri_pt0[1]) - (tri_pt1[1] - tri_pt0[1]) * (tri_pt2[0] - tri_pt0[0]);
        double a = _a / sqrt(pow(_a, 2) + pow(_b, 2) + pow(_c, 2));
        double b = _b / sqrt(pow(_a, 2) + pow(_b, 2) + pow(_c, 2));
        double c = _c / sqrt(pow(_a, 2) + pow(_b, 2) + pow(_c, 2));
        double d = 0 - (a * tri_pt0[0] + b * tri_pt0[1] + c * tri_pt0[2]);

        // distance can be calculated directly using the function, then we get the projected point as well
        double dist = fabs(a * pt[0] + b * pt[1] + c * pt[2] + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
        vec3d proj_pt;
        Plane p = Plane(a, b, c, d);
        short side = p.Side(pt, 1e-8);
        if (side == 1)
        {
            proj_pt[0] = pt[0] - a * dist;
            proj_pt[1] = pt[1] - b * dist;
            proj_pt[2] = pt[2] - c * dist;
        }
        else if (side == -1)
        {
            proj_pt[0] = pt[0] + a * dist;
            proj_pt[1] = pt[1] + b * dist;
            proj_pt[2] = pt[2] + c * dist;
        }
        else
        {
            proj_pt = pt;
        }

        // judge if the projected point is within the triangle
        // we calculate the cross product of each edge and the line with the point, to see if the results are all along the normal direction
        vec3d normal = CalFaceNormal(tri_pt0, tri_pt1, tri_pt2);
        vec3d AB, BC, CA, AP, BP, CP;
        AB[0] = tri_pt1[0] - tri_pt0[0];
        AB[1] = tri_pt1[1] - tri_pt0[1];
        AB[2] = tri_pt1[2] - tri_pt0[2];
        BC[0] = tri_pt2[0] - tri_pt1[0];
        BC[1] = tri_pt2[1] - tri_pt1[1];
        BC[2] = tri_pt2[2] - tri_pt1[2];
        CA[0] = tri_pt0[0] - tri_pt2[0];
        CA[1] = tri_pt0[1] - tri_pt2[1];
        CA[2] = tri_pt0[2] - tri_pt2[2];
        AP[0] = proj_pt[0] - tri_pt0[0];
        AP[1] = proj_pt[1] - tri_pt0[1];
        AP[2] = proj_pt[2] - tri_pt0[2];
        BP[0] = proj_pt[0] - tri_pt1[0];
        BP[1] = proj_pt[1] - tri_pt1[1];
        BP[2] = proj_pt[2] - tri_pt1[2];
        CP[0] = proj_pt[0] - tri_pt2[0];
        CP[1] = proj_pt[1] - tri_pt2[1];
        CP[2] = proj_pt[2] - tri_pt2[2];

        vec3d AB_AP = CrossProduct(AB, AP);
        vec3d BC_BP = CrossProduct(BC, BP);
        vec3d CA_CP = CrossProduct(CA, CP);

        // if all the cross product are parallel with normal, then the projected point is within the triangle
        // else, we should calculate the shortest distance to three edges
        if (SameVectorDirection(AB_AP, normal) && SameVectorDirection(BC_BP, normal) && SameVectorDirection(CA_CP, normal))
        {
            return dist;
        }
        else // if not within the triangle, we calculate the distance to 3 edges and 3 points and use the min
        {
            double dist_pt2AB = dist_point2segment(pt, tri_pt0, tri_pt1, flag);
            double dist_pt2BC = dist_point2segment(pt, tri_pt1, tri_pt2, flag);
            double dist_pt2CA = dist_point2segment(pt, tri_pt2, tri_pt0, flag);
            double dist_pt2A = dist_point2point(pt, tri_pt0);
            double dist_pt2B = dist_point2point(pt, tri_pt1);
            double dist_pt2C = dist_point2point(pt, tri_pt2);

            if (flag)
                cout << dist_pt2AB << ' ' << dist_pt2BC << ' ' << dist_pt2CA << ' '
                     << dist_pt2A << ' ' << dist_pt2B << ' ' << dist_pt2C << endl;

            return min(min(min(dist_pt2AB, dist_pt2BC), dist_pt2CA),
                       min(min(dist_pt2A, dist_pt2B), dist_pt2C));
        }
    }

    double face_hausdorff_distance(Model &meshA, vector<vec3d> &XA, vector<int> &idA, Model &meshB, vector<vec3d> &XB, vector<int> &idB, bool flag = false)
    {
        int nA = XA.size();
        int nB = XB.size();
        double cmax = 0;

        PointCloud<double> cloudA, cloudB;
        vec2PointCloud(cloudA, XA);
        vec2PointCloud(cloudB, XB);

        typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<double, PointCloud<double>>,
            PointCloud<double>,
            3 /* dim */
            >
            my_kd_tree_t;

        my_kd_tree_t indexA(3 /*dim*/, cloudA, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        my_kd_tree_t indexB(3 /*dim*/, cloudB, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        indexA.buildIndex();
        indexB.buildIndex();

        for (int i = 0; i < nB; i++)
        {
            size_t num_results = 10;
            double query_pt[3] = {XB[i][0], XB[i][1], XB[i][2]};

            std::vector<size_t> ret_index(num_results);
            std::vector<double> out_dist_sqr(num_results);

            num_results = indexA.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

            double cmin = INF;
            for (int j = 0; j < (int)num_results; j++)
            {
                double distance;
                distance = dist_point2triangle(XB[i], meshA.points[meshA.triangles[idA[ret_index[j]]][0]], meshA.points[meshA.triangles[idA[ret_index[j]]][1]], meshA.points[meshA.triangles[idA[ret_index[j]]][2]]);
                if (distance < cmin)
                {
                    cmin = distance;
                    if (cmin < 1e-14)
                        break;
                }
            }
            if (cmin > 10)
                cmin = sqrt(out_dist_sqr[0]);
            if (cmin > cmax && INF > cmin)
                cmax = cmin;
        }

        for (int i = 0; i < nA; i++)
        {
            size_t num_results = 10;

            double query_pt[3] = {XA[i][0], XA[i][1], XA[i][2]};

            std::vector<size_t> ret_index(num_results);
            std::vector<double> out_dist_sqr(num_results);

            num_results = indexB.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

            double cmin = INF;
            for (int j = 0; j < (int)num_results; j++)
            {
                double distance;
                distance = dist_point2triangle(XA[i], meshB.points[meshB.triangles[idB[ret_index[j]]][0]], meshB.points[meshB.triangles[idB[ret_index[j]]][1]], meshB.points[meshB.triangles[idB[ret_index[j]]][2]]);
                if (distance < cmin)
                {
                    cmin = distance;
                    if (cmin < 1e-14)
                        break;
                }
            }
            if (cmin > 10)
                cmin = sqrt(out_dist_sqr[0]);
            if (cmin > cmax && INF > cmin)
                cmax = cmin;
        }

        return cmax;
    }
}