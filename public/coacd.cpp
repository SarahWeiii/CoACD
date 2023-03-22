#include "coacd.h"
#include "../src/logger.h"
#include "../src/preprocess.h"
#include "../src/process.h"

namespace coacd {

void ManifoldPreprocess(Model &m, double scale = 30.0, double level_set = 0.55) {
  Model tmp = m;
  m.Clear();
  SDFManifold(tmp, m, scale, level_set);
}

void RecoverParts(vector<Model> &meshes, vector<double> bbox, array<array<double, 3>, 3> rot) {
  for (int i = 0; i < (int)meshes.size(); i++) {
    meshes[i].Recover(bbox);
    meshes[i].RevertPCA(rot);
  }
}

Mesh Remesh(Mesh const &input, int resolution, double level_set) {
  Model m;
  m.Load(input.vertices, input.indices);
  vector<double> bbox = m.Normalize();
  ManifoldPreprocess(m, resolution, level_set);
  m.Recover(bbox);

  return Mesh{.vertices = m.points, .indices = m.triangles};
}

std::vector<Mesh> CoACD(Mesh const &input, int downsampling, double threshold, int max_convex_hull, int resolution,
                        unsigned int seed, bool preprocess, int prep_resolution, bool pca, bool merge, int mcts_iteration,
                        int mcts_max_depth) {

  logger::info("threshold               {}", threshold);
  logger::info("max # convex hull       {}", max_convex_hull);
  logger::info("preprocess              {}", preprocess);
  logger::info("preprocess resolution   {}", prep_resolution);
  logger::info("pca                     {}", pca);
  logger::info("mcts max depth          {}", mcts_max_depth);
  logger::info("mcts nodes              {}", downsampling);
  logger::info("mcts iterations         {}", mcts_iteration);
  logger::info("merge                   {}", merge);
  logger::info("seed                    {}", seed);

  if (threshold < 0.01) {
    throw std::runtime_error("CoACD threshold < 0.01 (should be 0.01-1).");
  } else if (threshold > 1) {
    throw std::runtime_error("CoACD threshold > 1 (should be 0.01-1).");
  }

  if (prep_resolution > 1000) {
    throw std::runtime_error(
        "CoACD prep resolution > 1000, this is probably a bug (should be 30-100).");
  } else if (prep_resolution < 5) {
    throw std::runtime_error(
        "CoACD prep resolution < 5, this is probably a bug (should be 20-100).");
  }

  Model m;
  m.Load(input.vertices, input.indices);
  vector<double> bbox = m.Normalize();
  array<array<double, 3>, 3> rot{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  if (preprocess) {
    ManifoldPreprocess(m, prep_resolution);
  }
  if (pca) {
    rot = m.PCA();
  }

  Params params;
  params.input_model = "";
  params.output_name = "";
  params.downsampling = downsampling;
  params.threshold = threshold;
  params.resolution = resolution;
  params.seed = seed;
  params.preprocess = preprocess;
  params.prep_resolution = prep_resolution;
  params.pca = pca;
  params.merge = merge;
  params.mani_plus = false;
  params.mcts_iteration = mcts_iteration;
  params.mcts_max_depth = mcts_max_depth;

  vector<Model> parts = Compute(m, params);
  RecoverParts(parts, bbox, rot);

  std::vector<Mesh> result;
  for (auto &p : parts) {
    result.push_back(Mesh{.vertices = p.points, .indices = p.triangles});
  }
  return result;
}

void set_log_level(std::string_view level) {
  if (level == "off") {
    logger::get()->set_level(spdlog::level::off);
  } else if (level == "info") {
    logger::get()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    logger::get()->set_level(spdlog::level::warn);
  } else if (level == "error" || level == "err") {
    logger::get()->set_level(spdlog::level::err);
  } else if (level == "critical") {
    logger::get()->set_level(spdlog::level::critical);
  } else {
    throw std::runtime_error("invalid log level");
  }
}

} // namespace coacd