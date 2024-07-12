#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstring>

#include <math.h>
#include <limits>
#include <typeinfo>
#include <algorithm>
#include <assert.h>
#include <regex>

#include "shape.h"

namespace coacd
{
  class Params
  {
  public:
    /////////////// Basic Config ///////////////
    string input_model;
    string output_name;
    string remesh_output_name;
    int mcts_nodes;
    double threshold;
    unsigned int resolution;
    unsigned int seed;
    double rv_k;
    string preprocess_mode;
    int prep_resolution;
    bool pca;
    bool merge;
    int max_convex_hull;
    double dmc_thres;
    string apx_mode;

    /////////////// MCTS Config ///////////////
    int mcts_iteration;
    int mcts_max_depth;

    Params()
    {
      input_model = "../model.obj";
      output_name = "../output.obj";
      remesh_output_name = "../remesh.obj";
      mcts_nodes = 20;
      threshold = 0.05;
      resolution = 2000;
      seed = 1234;
      rv_k = 0.3;
      preprocess_mode = "auto";
      prep_resolution = 50;
      pca = false;
      merge = true;
      dmc_thres = 0.55;
      apx_mode = "ch";

      mcts_iteration = 150;
      mcts_max_depth = 3;
      max_convex_hull = -1;
    }
  };
}
