#include "./process.h"
#include "mcts.h"
#include "config.h"
#include "bvh.h"

#include <iostream>
#include <cmath>

namespace coacd
{
    thread_local std::mt19937 random_engine;

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
            {
                logger::info("\tWrong triangle orientation");
                end = clock();
                logger::info("Manifold Check Time: {}s", double(end - start) / CLOCKS_PER_SEC);
                return false;
            }
            if (!edge_num.contains({idx1, idx2}))
                edge_num[{idx1, idx2}] = 1;
            else
            {
                logger::info("\tWrong triangle orientation");
                end = clock();
                logger::info("Manifold Check Time: {}s", double(end - start) / CLOCKS_PER_SEC);
                return false;
            }
            if (!edge_num.contains({idx2, idx0}))
                edge_num[{idx2, idx0}] = 1;
            else
            {
                logger::info("\tWrong triangle orientation");
                end = clock();
                logger::info("Manifold Check Time: {}s", double(end - start) / CLOCKS_PER_SEC);
                return false;
            }
        }

        for (int i = 0; i < (int)edges.size(); i++)
        {
            pair<int, int> oppo_edge = {edges[i].second, edges[i].first};
            if (!edge_num.contains(oppo_edge))
            {
                logger::info("\tUnclosed mesh");
                end = clock();
                logger::info("Manifold Check Time: {}s", double(end - start) / CLOCKS_PER_SEC);
                return false;
            }
        }
        logger::info("[1/3] Edge check finish");

        // Check self-intersection
        BVH bvhTree(input);
        for (int i = 0; i < (int)input.triangles.size(); i++)
        {
            bool is_intersect = bvhTree.IntersectBVH(input.triangles[i], 0);
            if (is_intersect)
            {
                logger::info("\tTriangle self-intersection");
                end = clock();
                logger::info("Manifold Check Time: {}s", double(end - start) / CLOCKS_PER_SEC);
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

    double pts_norm(vec3d pt, vec3d p)
    {
        return sqrt(pow(pt[0] - p[0], 2) + pow(pt[1] - p[1], 2) + pow(pt[2] - p[2], 2));
    }

    double compute_edge_cost(Model &ch, string apx_mode, int tri_i, int tri_j, vector<int> &rm_pt_idxs)
    {
        // Compute the edge length
        double cost = pts_norm(ch.points[tri_i], ch.points[tri_j]);

        // // Compute the new merged point
        // vec3d tmp_pt = {0.5 * (ch.points[tri_i][0] + ch.points[tri_j][0]),
        //                 0.5 * (ch.points[tri_i][1] + ch.points[tri_j][1]),
        //                 0.5 * (ch.points[tri_i][2] + ch.points[tri_j][2])};

        // Model tmp_pts, tmp_ch;

        // // Use std::copy_if to filter out points instead of manually checking in a loop
        // std::copy_if(ch.points.begin(), ch.points.end(), std::back_inserter(tmp_pts.points),
        //             [&](const vec3d &pt) {
        //                 int idx = &pt - &ch.points[0]; // Get index
        //                 return idx != tri_i && idx != tri_j &&
        //                         std::find(rm_pt_idxs.begin(), rm_pt_idxs.end(), idx) == rm_pt_idxs.end();
        //             });

        // tmp_pts.points.push_back(tmp_pt);

        // // Compute the convex hull
        // tmp_pts.ComputeAPX(tmp_ch, apx_mode, true);

        // // Compute volume difference
        // double vol_diff = abs(MeshVolume(tmp_ch) - MeshVolume(ch));
        // std::cout << "vol_diff: " << vol_diff << std::endl;

        // // cost += vol_diff;
        return cost;
    }

    void DecimateCH(Model &ch, int tgt_pts, string apx_mode)
    {
        if (tgt_pts >= (int)ch.points.size())
            return;

        vector<vec3d> new_pts;
        vector<int> rm_pt_idxs;
        int n_pts = (int)ch.points.size();
        int tgt_n = min(tgt_pts, (int)ch.points.size());

        // compute edges
        vector<pair<double, pair<int, int>>> edge_costs;
        for (int i = 0; i < (int)ch.triangles.size(); i++)
        {
            for (int j = 0; j < 3; j++)
                if (ch.triangles[i][j] > ch.triangles[i][(j + 1) % 3])
                {
                    // double cost = pts_norm(ch.points[ch.triangles[i][j]], ch.points[ch.triangles[i][(j + 1) % 3]]);

                    double cost = compute_edge_cost(ch, apx_mode, ch.triangles[i][j], ch.triangles[i][(j + 1) % 3], rm_pt_idxs);

                    edge_costs.push_back({cost, {ch.triangles[i][j], ch.triangles[i][(j + 1) % 3]}});
                }
        }

        while (n_pts > tgt_n)
        {
            // sort the points by the cost
            sort(edge_costs.begin(), edge_costs.end());

            // remove the edge with the smallest cost
            pair<int, int> edge = edge_costs[0].second;
            vec3d new_pt = {0.5 * (ch.points[edge.first][0] + ch.points[edge.second][0]),
                            0.5 * (ch.points[edge.first][1] + ch.points[edge.second][1]),
                            0.5 * (ch.points[edge.first][2] + ch.points[edge.second][2])};

            rm_pt_idxs.push_back(edge.first);
            rm_pt_idxs.push_back(edge.second);

            ch.points.push_back(new_pt);
            n_pts -= 1;
            edge_costs[0].first = INF;

            // update the neighboring edge costs
            int new_pt_idx = ch.points.size() - 1;
            for (int i = 0; i < (int)edge_costs.size(); i++)
            {
                if (edge_costs[i].second.first == edge.first && edge_costs[i].second.second == edge.second)
                    edge_costs[i].first = INF;
                else if (edge_costs[i].second.first == edge.first || edge_costs[i].second.first == edge.second)
                {
                    // edge_costs[i].first = pts_norm(new_pt, ch.points[edge_costs[i].second.second]);
                    edge_costs[i].first = compute_edge_cost(ch, apx_mode, new_pt_idx, edge_costs[i].second.second, rm_pt_idxs);
                    edge_costs[i].second.first = new_pt_idx;
                }
                else if (edge_costs[i].second.second == edge.first || edge_costs[i].second.second == edge.second)
                {
                    // edge_costs[i].first = pts_norm(new_pt, ch.points[edge_costs[i].second.first]);
                    edge_costs[i].first = compute_edge_cost(ch, apx_mode, edge_costs[i].second.first, new_pt_idx, rm_pt_idxs);
                    edge_costs[i].second.second = edge_costs[i].second.first;
                    edge_costs[i].second.first = new_pt_idx; // larger idx should be the first!
                }
            }
        }

        // remove the points and add new points
        Model new_ch;
        for (int i = 0; i < (int)ch.points.size(); i++)
        {
            if (find(rm_pt_idxs.begin(), rm_pt_idxs.end(), i) == rm_pt_idxs.end())
                new_ch.points.push_back(ch.points[i]);
        }

        new_ch.ComputeAPX(ch, apx_mode, true);
    }

    void DecimateConvexHulls(vector<Model> &cvxs, Params &params)
    {
        logger::info(" - Simplify Convex Hulls");
        for (int i = 0; i < (int)cvxs.size(); i++)
        {
            DecimateCH(cvxs[i], params.max_ch_vertex, params.apx_mode);
        }
    }

    void MergeCH(Model &ch1, Model &ch2, Model &ch, Params &params)
    {
        Model merge;
        merge.points.insert(merge.points.end(), ch1.points.begin(), ch1.points.end());
        merge.points.insert(merge.points.end(), ch2.points.begin(), ch2.points.end());
        merge.triangles.insert(merge.triangles.end(), ch1.triangles.begin(), ch1.triangles.end());
        for (int i = 0; i < (int)ch2.triangles.size(); i++)
            merge.triangles.push_back({int(ch2.triangles[i][0] + ch1.points.size()),
                                       int(ch2.triangles[i][1] + ch1.points.size()), int(ch2.triangles[i][2] + ch1.points.size())});
        merge.ComputeAPX(ch, params.apx_mode, true);
    }

    double MergeConvexHulls(Model &m, vector<Model> &meshs, vector<Model> &cvxs, Params &params, double epsilon, double threshold)
    {
        logger::info(" - Merge Convex Hulls");
        size_t nConvexHulls = (size_t)cvxs.size();
        double h = 0;

        if (nConvexHulls > 1)
        {
            int bound = ((((nConvexHulls - 1) * nConvexHulls)) >> 1);
            // Populate the cost matrix
            vector<double> costMatrix, precostMatrix;
            costMatrix.resize(bound);    // only keeps the top half of the matrix
            precostMatrix.resize(bound); // only keeps the top half of the matrix

            size_t p1, p2;
#ifdef _OPENMP
#pragma omp parallel for default(none) shared(costMatrix, precostMatrix, cvxs, params, bound, threshold, meshs) private(p1, p2)
#endif
            for (int idx = 0; idx < bound; ++idx)
            {
                p1 = (int)(sqrt(8 * idx + 1) - 1) >> 1; // compute nearest triangle number index
                int sum = (p1 * (p1 + 1)) >> 1;         // compute nearest triangle number from index
                p2 = idx - sum;                         // modular arithmetic from triangle number
                p1++;
                double dist = MeshDist(cvxs[p1], cvxs[p2]);
                if (dist < threshold)
                {
                    Model combinedCH;
                    MergeCH(cvxs[p1], cvxs[p2], combinedCH, params);

                    costMatrix[idx] = ComputeHCost(cvxs[p1], cvxs[p2], combinedCH, params.rv_k, params.resolution, params.seed);
                    precostMatrix[idx] = max(ComputeHCost(meshs[p1], cvxs[p1], params.rv_k, 3000, params.seed),
                                             ComputeHCost(meshs[p2], cvxs[p2], params.rv_k, 3000, params.seed));
                }
                else
                {
                    costMatrix[idx] = INF;
                }
            }

            size_t costSize = (size_t)cvxs.size();

            while (true)
            {
                // Search for lowest cost
                double bestCost = INF;
                const size_t addr = FindMinimumElement(costMatrix, &bestCost, 0, (int32_t)costMatrix.size());

                if (params.max_convex_hull <= 0)
                {
                    // if dose not set max nConvexHull, stop the merging when bestCost is larger than the threshold
                    if (bestCost > params.threshold)
                        break;
                    if (bestCost > max(params.threshold - precostMatrix[addr], 0.01)) // avoid merging two parts that have already used up the treshold
                    {
                        costMatrix[addr] = INF;
                        continue;
                    }
                }
                else
                {
                    // if set the max nConvexHull, ignore the threshold limitation and stio the merging untill # part reach the constraint
                    if ((int)cvxs.size() <= params.max_convex_hull && bestCost > params.threshold)
                    {
                        if (bestCost > params.threshold + 0.005 && (int)cvxs.size() == params.max_convex_hull)
                            logger::warn("Max concavity {} exceeds the threshold {} due to {} convex hull limitation", bestCost, params.threshold, params.max_convex_hull);
                        break;
                    }
                    if ((int)cvxs.size() <= params.max_convex_hull && bestCost > max(params.threshold - precostMatrix[addr], 0.01)) // avoid merging two parts that have already used up the treshold
                    {
                        costMatrix[addr] = INF;
                        continue;
                    }
                }

                h = max(h, bestCost);
                const size_t addrI = (static_cast<int32_t>(sqrt(1 + (8 * addr))) - 1) >> 1;
                const size_t p1 = addrI + 1;
                const size_t p2 = addr - ((addrI * (addrI + 1)) >> 1);
                // printf("addr %ld, addrI %ld, p1 %ld, p2 %ld\n", addr, addrI, p1, p2);
                assert(p1 >= 0);
                assert(p2 >= 0);
                assert(p1 < costSize);
                assert(p2 < costSize);

                // Make the lowest cost row and column into a new hull
                Model cch;
                MergeCH(cvxs[p1], cvxs[p2], cch, params);
                cvxs[p2] = cch;

                std::swap(cvxs[p1], cvxs[cvxs.size() - 1]);
                cvxs.pop_back();

                costSize = costSize - 1;

                // Calculate costs versus the new hull
                size_t rowIdx = ((p2 - 1) * p2) >> 1;
                for (size_t i = 0; (i < p2); ++i)
                {
                    double dist = MeshDist(cvxs[p2], cvxs[i]);
                    if (dist < threshold)
                    {
                        Model combinedCH;
                        MergeCH(cvxs[p2], cvxs[i], combinedCH, params);
                        costMatrix[rowIdx] = ComputeHCost(cvxs[p2], cvxs[i], combinedCH, params.rv_k, params.resolution, params.seed);
                        precostMatrix[rowIdx++] = max(precostMatrix[p2] + bestCost, precostMatrix[i]);
                    }
                    else
                        costMatrix[rowIdx++] = INF;
                }

                rowIdx += p2;
                for (size_t i = p2 + 1; (i < costSize); ++i)
                {
                    double dist = MeshDist(cvxs[p2], cvxs[i]);
                    if (dist < threshold)
                    {
                        Model combinedCH;
                        MergeCH(cvxs[p2], cvxs[i], combinedCH, params);
                        costMatrix[rowIdx] = ComputeHCost(cvxs[p2], cvxs[i], combinedCH, params.rv_k, params.resolution, params.seed);
                        precostMatrix[rowIdx] = max(precostMatrix[p2] + bestCost, precostMatrix[i]);
                    }
                    else
                        costMatrix[rowIdx] = INF;
                    rowIdx += i;
                    assert(rowIdx >= 0);
                }

                // Move the top column in to replace its space
                const size_t erase_idx = ((costSize - 1) * costSize) >> 1;
                if (p1 < costSize)
                {
                    rowIdx = (addrI * p1) >> 1;
                    size_t top_row = erase_idx;
                    for (size_t i = 0; i < p1; ++i)
                    {
                        if (i != p2)
                        {
                            costMatrix[rowIdx] = costMatrix[top_row];
                            precostMatrix[rowIdx] = precostMatrix[top_row];
                        }
                        ++rowIdx;
                        ++top_row;
                    }

                    ++top_row;
                    rowIdx += p1;
                    for (size_t i = p1 + 1; i < costSize; ++i)
                    {
                        costMatrix[rowIdx] = costMatrix[top_row];
                        precostMatrix[rowIdx] = precostMatrix[top_row++];
                        rowIdx += i;
                    }
                }
                costMatrix.resize(erase_idx);
                precostMatrix.resize(erase_idx);
            }
        }

        return h;
    }

    void ExtrudeCH(Model &ch, Plane overlap_plane, Params &params, double margin)
    {
        vec3d normal = {overlap_plane.a, overlap_plane.b, overlap_plane.c};

        // decide the extrude direction by other points of the ch
        int side = 0;
        for (int i = 0; i < (int)ch.points.size(); i++)
        {
            vec3d p = ch.points[i];
            side += overlap_plane.Side(p, 1e-4);
        }
        side = side > 0 ? 1 : -1;

        for (int i = 0; i < (int)ch.points.size(); i++)
        {
            if (overlap_plane.Side(ch.points[i], 1e-4) == 0)
                ch.points[i] = {ch.points[i][0] - side * margin * normal[0],
                                ch.points[i][1] - side * margin * normal[1],
                                ch.points[i][2] - side * margin * normal[2]};
        }

        Model tmp;
        ch.ComputeAPX(tmp, params.apx_mode, true);
        ch = tmp;
    }

    void ExtrudeConvexHulls(vector<Model> &cvxs, Params &params, double eps)
    {
        logger::info(" - Extrude Convex Hulls");
        for (int i = 0; i < (int)cvxs.size(); i++)
        {
            Model cvx = cvxs[i];
            for (int j = 0; j < (int)cvxs.size(); j++)
            {
                Model convex1 = cvxs[i], convex2 = cvxs[j];
                Plane overlap_plane;

                double dist = MeshDist(convex1, convex2);
                bool flag = ComputeOverlapFace(convex1, convex2, overlap_plane);

                // only extrude the convex hulls along the normal of overlap plane
                if (dist < eps && flag)
                {
                    ExtrudeCH(convex1, overlap_plane, params, params.extrude_margin);
                    cvxs[i] = convex1;
                    ExtrudeCH(convex2, overlap_plane, params, params.extrude_margin);
                    cvxs[j] = convex2;
                }
            }
        }
    }

    vector<Model> Compute(Model &mesh, Params &params)
    {
        vector<Model> InputParts = {mesh};
        vector<Model> parts, pmeshs;
#ifdef _OPENMP
        omp_lock_t writelock;
        omp_init_lock(&writelock);
        double start, end;
        start = omp_get_wtime();
#else
        clock_t start, end;
        start = clock();
#endif

        logger::info("# Points: {}", mesh.points.size());
        logger::info("# Triangles: {}", mesh.triangles.size());
        logger::info(" - Decomposition (MCTS)");

        size_t iter = 0;
        double cut_area;
        while ((int)InputParts.size() > 0)
        {
            vector<Model> tmp;
            logger::info("iter {} ---- waiting pool: {}", iter, InputParts.size());
#ifdef _OPENMP
#pragma omp parallel for default(none) shared(InputParts, params, mesh, writelock, parts, pmeshs, tmp) private(cut_area)
#endif
            for (int p = 0; p < (int)InputParts.size(); p++)
            {
                random_engine.seed(params.seed);
                if (p % ((int)InputParts.size() / 10 + 1) == 0)
                    logger::info("Processing [{:.1f}%]", p * 100.0 / (int)InputParts.size());

                Model pmesh = InputParts[p], pCH;
                Plane bestplane;
                pmesh.ComputeAPX(pCH, params.apx_mode, true);
                double h = ComputeHCost(pmesh, pCH, params.rv_k, params.resolution, params.seed, 0.0001, false);

                if (h > params.threshold)
                {
                    vector<Plane> planes, best_path;

                    // MCTS for cutting plane
                    Node *node = new Node(params);
                    State state(params, pmesh);
                    node->set_state(state);
                    Node *best_next_node = MonteCarloTreeSearch(params, node, best_path);
                    if (best_next_node == NULL)
                    {
#ifdef _OPENMP
                        omp_set_lock(&writelock);
#endif
                        parts.push_back(pCH);
                        pmeshs.push_back(pmesh);
                        free_tree(node, 0);
#ifdef _OPENMP
                        omp_unset_lock(&writelock);
#endif
                    }
                    else
                    {
                        bestplane = best_next_node->state->current_value.first;
                        TernaryMCTS(pmesh, params, bestplane, best_path, best_next_node->quality_value); // using Rv to Ternary refine
                        free_tree(node, 0);

                        Model pos, neg;
                        bool clipf = Clip(pmesh, pos, neg, bestplane, cut_area);
                        if (!clipf)
                        {
                            logger::error("Wrong clip proposal!");
                            exit(0);
                        }
#ifdef _OPENMP
                        omp_set_lock(&writelock);
#endif
                        if ((int)pos.triangles.size() > 0)
                            tmp.push_back(pos);
                        if ((int)neg.triangles.size() > 0)
                            tmp.push_back(neg);
#ifdef _OPENMP
                        omp_unset_lock(&writelock);
#endif
                    }
                }
                else
                {
#ifdef _OPENMP
                    omp_set_lock(&writelock);
#endif
                    parts.push_back(pCH);
                    pmeshs.push_back(pmesh);
#ifdef _OPENMP
                    omp_unset_lock(&writelock);
#endif
                }
            }
            logger::info("Processing [100.0%]");
            InputParts.clear();
            InputParts = tmp;
            tmp.clear();
            iter++;
        }
        if (params.merge)
            MergeConvexHulls(mesh, pmeshs, parts, params);

        if (params.decimate)
            DecimateConvexHulls(parts, params);

        if (params.extrude)
            ExtrudeConvexHulls(parts, params);

#ifdef _OPENMP
        end = omp_get_wtime();
        logger::info("Compute Time: {}s", double(end - start));
#else
        end = clock();
        logger::info("Compute Time: {}s", double(end - start) / CLOCKS_PER_SEC);
#endif
        logger::info("# Convex Hulls: {}", (int)parts.size());

        return parts;
    }
}
