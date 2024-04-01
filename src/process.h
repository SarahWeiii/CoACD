#pragma once

#include <iostream>
#include <string>
#include <random>
#include <fstream>
#include <vector>
#include <math.h>
#include <limits>
#include <typeinfo>
#include <algorithm>
#include <assert.h>
#include <regex>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "./io.h"
#include "clip.h"
#include "config.h"
#include "model_obj.h"
#include "cost.h"

namespace coacd
{
  extern thread_local std::mt19937 random_engine;

  void MergeCH(Model &ch1, Model &ch2, Model &ch);
  double MergeConvexHulls(Model &m, vector<Model> &meshs, vector<Model> &cvxs, Params &params, double epsilon = 0.02, double threshold = 0.01);
  vector<Model> Compute(Model &mesh, Params &params);
  bool IsManifold(Model &input);

  inline void addNeighbor(map<pair<int, int>, pair<int, int>> &edge_map, pair<int, int> &edge, vector<int> &neighbors, int idx)
  {
    int first = edge_map[edge].first;
    int second = edge_map[edge].second;
    if (first != idx && first != -1)
      neighbors.push_back(first);
    if (second != idx && second != -1)
      neighbors.push_back(second);
  }

  inline int32_t FindMinimumElement(const vector<double> d, double *const m, const int32_t begin, const int32_t end)
  {
    int32_t idx = -1;
    double min = (std::numeric_limits<double>::max)();
    for (size_t i = begin; i < size_t(end); ++i)
    {
      if (d[i] < min)
      {
        idx = i;
        min = d[i];
      }
    }

    *m = min;
    return idx;
  }
}