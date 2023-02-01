#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <time.h>

#include "src/process.h"
#include "src/logger.h"

using namespace std;
using namespace coacd;

int main(int argc, char *argv[])
{
  Params params;

  // Model files
  string input_model;
  params.seed = (unsigned)time(NULL);

  // args
  for (int i = 0; i < argc; ++i)
  {
    if (i < argc)
    {
      if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--input") == 0)
      {
        params.input_model = argv[i + 1];
      }
      if (strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--output") == 0)
      {
        params.output_name = argv[i + 1];
      }
      if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--log") == 0)
      {
        params.logfile = argv[i + 1];
      }
      if (strcmp(argv[i], "-k") == 0)
      {
        sscanf(argv[i + 1], "%lf", &params.rv_k);
      }
      if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--threshold") == 0)
      {
        sscanf(argv[i + 1], "%lf", &params.threshold);
      }
      if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--resolution") == 0)
      {
        sscanf(argv[i + 1], "%u", &params.resolution);
      }
      if (strcmp(argv[i], "-np") == 0 || strcmp(argv[i], "--no-prerpocess") == 0)
      {
        params.preprocess = false;
      }
      if (strcmp(argv[i], "-nm") == 0 || strcmp(argv[i], "--no-merge") == 0)
      {
        params.merge = false;
      }
      if (strcmp(argv[i], "-nmp") == 0 || strcmp(argv[i], "--no-manifold-plus") == 0)
      {
        params.mani_plus = false;
      }
      if (strcmp(argv[i], "-nc") == 0 || strcmp(argv[i], "--no-cout") == 0)
      {
        params.if_cout = false;
      }
      if (strcmp(argv[i], "-nl") == 0 || strcmp(argv[i], "--no-log") == 0)
      {
        params.if_log = false;
      }
      if (strcmp(argv[i], "--pca") == 0)
      {
        params.pca = true;
      }
      if (strcmp(argv[i], "-pr") == 0 || strcmp(argv[i], "--prep-resolution") == 0)
      {
        sscanf(argv[i + 1], "%d", &params.prep_resolution);
      }
      if (strcmp(argv[i], "-pd") == 0 || strcmp(argv[i], "--prep-depth") == 0)
      {
        sscanf(argv[i + 1], "%d", &params.prep_depth);
      }
      if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--seed") == 0)
      {
        sscanf(argv[i + 1], "%u", &params.seed);
      }

      if (strcmp(argv[i], "-mi") == 0 || strcmp(argv[i], "--mcts-iteration") == 0)
      {
        sscanf(argv[i + 1], "%d", &params.mcts_iteration);
      }
      if (strcmp(argv[i], "-md") == 0 || strcmp(argv[i], "--mcts-depth") == 0)
      {
        sscanf(argv[i + 1], "%d", &params.mcts_max_depth);
      }
      if (strcmp(argv[i], "-mn") == 0 || strcmp(argv[i], "--mcts-node") == 0)
      {
        sscanf(argv[i + 1], "%d", &params.downsampling);
      }
    }
  }

  string ext;
  if (params.input_model.length() > 4)
  {
    ext = params.input_model.substr(params.input_model.length() - 4);
    if (ext != ".obj")
    {
      logger(params.if_cout, false) << "Error: Input must be OBJ format!" << endl;
      exit(0);
    }
  }
  else
  {
    logger(params.if_cout, false) << "Error: Input Filename Error!" << endl;
    exit(0);
  }

  if (params.output_name.length() > 4)
    ext = params.output_name.substr(params.output_name.length() - 4);
  else
  {
    logger(params.if_cout, false) << "Error: Output Filename Error! You can set the output filename as either .OBJ or .WRL!" << endl;
    exit(0);
  }
  if (params.logfile == "")
  {

    if (ext == ".obj")
      params.logfile = regex_replace(params.output_name, regex(".obj"), "_log.txt");
    else if (ext == ".wrl")
      params.logfile = regex_replace(params.output_name, regex(".wrl"), "_log.txt");
    else
    {
      logger(params.if_cout, false) << "Error: Output Filename must be .OBJ or .WRL format!" << endl;
      exit(0);
    }
  }

  if (params.threshold < 0.01)
    logger(params.if_cout, false) << "Warning: Threshold t exceeds the lower bound and is automatically set as 0.01!" << endl;
  else if (params.threshold > 1)
    logger(params.if_cout, false) << "Warning: Threshold t exceeds the higher bound and is automatically set as 1!" << endl;
  params.threshold = min(max(params.threshold, 0.01), 1.0);

  Model m, n, pos, neg;

  SaveConfig(params);

  m.LoadOBJ(params.input_model);
  m.Normalize();
  if (params.preprocess)
    ManifoldPreprocess(params, m);
  if (params.pca)
    m.PCA();

  Compute(m, params);

  return 0;
}
