#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <math.h>
#include <limits>
#include <typeinfo>
#include <algorithm>
#include <assert.h>
#include <regex>

#include "io.h"
#include "clip.h"
#include "config.h"
#include "model_obj.h"
#include "cost.h"

namespace coacd
{

  void ManifoldPreprocess(Params &params, Model &m);
  void MergeCH(Model &ch1, Model &ch2, Model &ch);
  double MergeConvexHulls(Model &m, std::vector<Model> &meshs, std::vector<Model> &cvxs, Params &params, double epsilon = 0.02, double threshold = 0.01);
  std::vector<Model> Compute(Model &mesh, Params &params);

  inline void addNeighbor(std::map<std::pair<int, int>, std::pair<int, int>> &edge_map, std::pair<int, int> &edge, std::vector<int> &neighbors, int idx)
  {
    int first = edge_map[edge].first;
    int second = edge_map[edge].second;
    if (first != idx && first != -1)
      neighbors.push_back(first);
    if (second != idx && second != -1)
      neighbors.push_back(second);
  }

  inline int32_t FindMinimumElement(const std::vector<double> d, double *const m, const int32_t begin, const int32_t end)
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