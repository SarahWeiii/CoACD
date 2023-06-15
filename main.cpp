#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <time.h>

#include "src/preprocess.h"
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
      if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--max-convex-hull") == 0)
      {
        sscanf(argv[i + 1], "%u", &params.max_convex_hull);
      }
      if (strcmp(argv[i], "-nm") == 0 || strcmp(argv[i], "--no-merge") == 0)
      {
        params.merge = false;
      }
      if (strcmp(argv[i], "--pca") == 0)
      {
        params.pca = true;
      }
      if (strcmp(argv[i], "-pm") == 0 || strcmp(argv[i], "--preprocess-mode") == 0)
      {
        params.preprocess_mode = argv[i + 1];
      }
      if (strcmp(argv[i], "-pr") == 0 || strcmp(argv[i], "--prep-resolution") == 0)
      {
        sscanf(argv[i + 1], "%d", &params.prep_resolution);
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
        sscanf(argv[i + 1], "%d", &params.mcts_nodes);
      }
    }
  }

  string ext;
  if (params.input_model.length() > 4)
  {
    ext = params.input_model.substr(params.input_model.length() - 4);
    if (ext != ".obj")
    {
      logger::critical("Input must be OBJ format!");
      exit(0);
    }
  }
  else
  {
    logger::critical("Input Filename Error!");
    exit(0);
  }

  if (params.output_name.length() > 4)
    ext = params.output_name.substr(params.output_name.length() - 4);
  else
  {
    logger::critical("Output Filename Error! You can set the output filename as either .OBJ or .WRL!");
    exit(0);
  }
  if (ext != ".obj" && ext != ".wrl")
  {
    logger::critical("Output Filename must be .OBJ or .WRL format!");
    exit(0);
  }

  if (params.threshold < 0.01)
    logger::warn("Threshold t exceeds the lower bound and is automatically set as 0.01!");
  else if (params.threshold > 1)
    logger::warn("Threshold t exceeds the higher bound and is automatically set as 1!");
  params.threshold = min(max(params.threshold, 0.01), 1.0);

  Model m;
  array<array<double, 3>, 3> rot;

  SaveConfig(params);

  m.LoadOBJ(params.input_model);
  vector<double> bbox = m.Normalize();
  m.SaveOBJ("normalized.obj");

  if (params.preprocess_mode == "auto")
  {
    bool is_manifold = IsManifold(m);
    logger::info("Mesh Manifoldness: {}", is_manifold);
    if (!is_manifold)
      ManifoldPreprocess(params, m);
  }
  else if (params.preprocess_mode == "on")
    ManifoldPreprocess(params, m);

  if (params.pca)
    rot = m.PCA();

  vector<Model> parts = Compute(m, params);

  RecoverParts(parts, bbox, rot, params);

  string objName = regex_replace(params.output_name, regex("wrl"), "obj");
  string wrlName = regex_replace(params.output_name, regex("obj"), "wrl");

  SaveVRML(wrlName, parts, params);
  SaveOBJ(objName, parts, params);

  return 0;
}
