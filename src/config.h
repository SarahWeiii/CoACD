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

#pragma once

using namespace std;

class Params
{
public:
  /////////////// Basic Config ///////////////
  string input_model;
  string output_name;
  string logfile;
  string mode;
  int downsampling;
  double threshold;
  unsigned int resolution;
  unsigned int seed;
  double rv_k;
  bool preprocess;
  int prep_resolution;
  int prep_depth;
  bool pca;
  bool merge;
  bool mani_plus;

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
    prep_depth = 8;
    pca = false;
    merge = true;
    mani_plus = true;

    mcts_iteration = 150;
    mcts_max_depth = 3;
  }
};
