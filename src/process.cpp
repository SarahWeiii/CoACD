#include "process.h"
#include "mcts.h"
#include "manifold/Manifold.h"
#include "manifold/ManifoldPlus.h"
#include "config.h"
#include <iostream>
#include <cmath>

namespace coacd
{

    void ManifoldPreprocess(Params &params, Model &m)
    {
        Model tmp = m;
        // tmp.LoadOBJ(params.input_model);
        bool is_thin = m.CheckThin();
        m.Clear();

        if (params.mani_plus && is_thin)
            ManifoldPlus(tmp, m, params.if_cout, params.if_log, params.logfile, params.prep_depth);
            // ManifoldPlus(params.input_model, m, params.prep_depth);
        else
            Manifold(tmp, m, params.if_cout, params.if_log, params.logfile, params.prep_resolution);
            // Manifold(params.input_model, m, params.prep_resolution);

        // TODO: not elegant
        m.m_len = tmp.m_len;
        m.m_Xmid = tmp.m_Xmid;
        m.m_Ymid = tmp.m_Ymid;
        m.m_Zmid = tmp.m_Zmid;
        
    }

    void MergeCH(Model &ch1, Model &ch2, Model &ch)
    {
        Model merge;
        merge.points.insert(merge.points.end(), ch1.points.begin(), ch1.points.end());
        merge.points.insert(merge.points.end(), ch2.points.begin(), ch2.points.end());
        merge.triangles.insert(merge.triangles.end(), ch1.triangles.begin(), ch1.triangles.end());
        for (int i = 0; i < (int)ch2.triangles.size(); i++)
            merge.triangles.push_back({int(ch2.triangles[i][0] + ch1.points.size()),
                                       int(ch2.triangles[i][1] + ch1.points.size()), int(ch2.triangles[i][2] + ch1.points.size())});
        merge.ComputeCH(ch);
        SyncNorm(ch1, ch);
    }

    double MergeConvexHulls(Model &m, vector<Model> &meshs, vector<Model> &cvxs, Params &params, double epsilon, double threshold)
    {
        logger(params.if_cout, false) << " - Merge Convex Hulls" << endl;
        size_t nConvexHulls = (size_t)cvxs.size();
        double h = 0;

        if (nConvexHulls > 1)
        {
            int bound = ((((nConvexHulls - 1) * nConvexHulls)) >> 1);
            // Populate the cost matrix
            size_t idx = 0;
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
                    MergeCH(cvxs[p1], cvxs[p2], combinedCH);

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
                if (bestCost > params.threshold)
                    break;
                if (bestCost > max(params.threshold - precostMatrix[addr], 0.01))
                {
                    costMatrix[addr] = INF;
                    continue;
                }
                h = max(h, bestCost);
                const size_t addrI = (static_cast<int32_t>(sqrt(1 + (8 * addr))) - 1) >> 1;
                const size_t p1 = addrI + 1;
                const size_t p2 = addr - ((addrI * (addrI + 1)) >> 1);
                assert(p1 >= 0);
                assert(p2 >= 0);
                assert(p1 < costSize);
                assert(p2 < costSize);

                // Make the lowest cost row and column into a new hull
                Model cch;
                MergeCH(cvxs[p1], cvxs[p2], cch);
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
                        MergeCH(cvxs[p2], cvxs[i], combinedCH);
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
                        MergeCH(cvxs[p2], cvxs[i], combinedCH);
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
                    for (size_t i = p1 + 1; i < (costSize + 1); ++i)
                    {
                        costMatrix[rowIdx] = costMatrix[top_row];
                        precostMatrix[rowIdx] = precostMatrix[top_row++];
                        rowIdx += i;
                        assert(rowIdx >= 0);
                    }
                }
                costMatrix.resize(erase_idx);
                precostMatrix.resize(erase_idx);
            }
        }

        return h;
    }

    void Compute(Model &mesh, Params &params)
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

        logger(false, params.if_log, params.logfile) << "#Points: " << mesh.points.size() << endl;
        logger(false, params.if_log, params.logfile) << "#Triangles: " << mesh.triangles.size() << endl;

        logger(false, params.if_log, params.logfile) << " - Decomposition (MCTS)" << endl;
        logger(params.if_cout, false) << " - Decomposition (MCTS)" << endl;

        size_t iter = 0;
        double cut_area;
        while ((int)InputParts.size() > 0)
        {
            vector<Model> tmp;
            logger(false, params.if_log, params.logfile) << "iter " << iter << " ---- "
                                                         << "waiting pool: " << InputParts.size() << endl;
            logger(params.if_cout, false) << "iter " << iter << " ---- "
                                          << "waiting pool: " << InputParts.size() << endl;
#ifdef _OPENMP
#pragma omp parallel for default(none) shared(InputParts, params, mesh, writelock, parts, pmeshs, tmp) private(cut_area)
#endif
            for (int p = 0; p < (int)InputParts.size(); p++)
            {
                if (p % ((int)InputParts.size() / 10 + 1) == 0)
                    logger(params.if_cout, false) << "Processing [" << p * 100.0 / (int)InputParts.size() << "%]" << endl;

                Model pmesh = InputParts[p], pCH;
                Plane bestplane;
                pmesh.ComputeVCH(pCH);
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
                        SyncNorm(mesh, pCH);
#ifdef _OPENMP
                        omp_set_lock(&writelock);
#endif
                        parts.push_back(pCH);
                        pmeshs.push_back(pmesh);
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
                            logger(params.if_cout, false) << "Wrong clip proposal!" << endl;
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
                    SyncNorm(mesh, pCH);
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
            logger(params.if_cout, false) << "Processing [100%]" << endl;
            InputParts.clear();
            InputParts = tmp;
            tmp.clear();
            iter++;
        }
        if (params.merge)
            MergeConvexHulls(mesh, pmeshs, parts, params);

#ifdef _OPENMP
        end = omp_get_wtime();
        logger(false, params.if_log, params.logfile) << "Compute Time: " << double(end - start) << "s" << endl;
        logger(params.if_cout, false) << "Compute Time: " << double(end - start) << "s" << endl;
#else
        end = clock();
        logger(false, params.if_log, params.logfile) << "Compute Time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
        logger(params.if_cout, false) << "Compute Time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
#endif

        logger(false, params.if_log, params.logfile) << "#Convex After Merge: " << (int)parts.size() << endl;
        logger(params.if_cout, false) << "#Convex After Merge: " << (int)parts.size() << endl;

        string objName = regex_replace(params.output_name, regex("wrl"), "obj");
        string wrlName = regex_replace(params.output_name, regex("obj"), "wrl");

        SaveVRML(wrlName, parts, params);
        SaveOBJ(objName, parts, params);
    }
}