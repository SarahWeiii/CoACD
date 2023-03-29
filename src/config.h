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
    std::string input_model;
    std::string output_name;
    std::string logfile;
    std::string mode;
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
    bool sdf_mani;
    bool if_cout;
    bool if_log;

    /////////////// MCTS Config ///////////////
    int mcts_iteration;
    int mcts_max_depth;

    Params()
    {
      input_model = "../model.obj";
      output_name = "../output.obj";
      logfile = "";
      mode = "custom";
      downsampling = 20;
      threshold = 0.05;
      resolution = 2000;
      seed = 1234;
      rv_k = 0.3;
      preprocess = true;
      prep_resolution = 10000;
      pca = false;
      merge = true;
      mani_plus = true;
      sdf_mani = false;
      if_cout = true;
      if_log = true;

      mcts_iteration = 150;
      mcts_max_depth = 3;
    }
  };
}