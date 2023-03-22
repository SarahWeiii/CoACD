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

using namespace std;

namespace coacd
{
  class Params
  {
  public:
    /////////////// Basic Config ///////////////
    string input_model;
    string output_name;
    int downsampling;
    double threshold;
    unsigned int resolution;
    unsigned int seed;
    double rv_k;
    bool preprocess;
    int prep_resolution;
    bool pca;
    bool merge;
    bool mani_plus;
    int max_convex_hull;

    /////////////// MCTS Config ///////////////
    int mcts_iteration;
    int mcts_max_depth;

    Params()
    {
      input_model = "../model.obj";
      output_name = "../output.obj";
      downsampling = 20;
      threshold = 0.05;
      resolution = 2000;
      seed = 1234;
      rv_k = 0.3;
      preprocess = true;
      prep_resolution = 50;
      pca = false;
      merge = true;
      mani_plus = true;

      mcts_iteration = 150;
      mcts_max_depth = 3;
      max_convex_hull = -1;
    }
  };
}