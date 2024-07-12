#include "coacd.h"
#include "../src/logger.h"
#include "../src/preprocess.h"
#include "../src/process.h"

namespace coacd {
void RecoverParts(vector<Model> &meshes, vector<double> bbox,
                  array<array<double, 3>, 3> rot) {
  for (int i = 0; i < (int)meshes.size(); i++) {
    meshes[i].Recover(bbox);
    meshes[i].RevertPCA(rot);
  }
}

std::vector<Mesh> CoACD(Mesh const &input, double threshold,
                        int max_convex_hull, std::string preprocess_mode,
                        int prep_resolution, int sample_resolution,
                        int mcts_nodes, int mcts_iteration, int mcts_max_depth,
                        bool pca, bool merge, std::string apx_mode, unsigned int seed) {

  logger::info("threshold               {}", threshold);
  logger::info("max # convex hull       {}", max_convex_hull);
  logger::info("preprocess mode         {}", preprocess_mode);
  logger::info("preprocess resolution   {}", prep_resolution);
  logger::info("pca                     {}", pca);
  logger::info("mcts max depth          {}", mcts_max_depth);
  logger::info("mcts nodes              {}", mcts_nodes);
  logger::info("mcts iterations         {}", mcts_iteration);
  logger::info("merge                   {}", merge);
  logger::info("approximate mode        {}", apx_mode);
  logger::info("seed                    {}", seed);

  if (threshold < 0.01) {
    throw std::runtime_error("CoACD threshold < 0.01 (should be 0.01-1).");
  } else if (threshold > 1) {
    throw std::runtime_error("CoACD threshold > 1 (should be 0.01-1).");
  }

  if (prep_resolution > 1000) {
    throw std::runtime_error("CoACD prep resolution > 1000, this is probably a "
                             "bug (should be 30-100).");
  } else if (prep_resolution < 5) {
    throw std::runtime_error("CoACD prep resolution < 5, this is probably a "
                             "bug (should be 20-100).");
  }

  Params params;
  params.input_model = "";
  params.output_name = "";
  params.threshold = threshold;
  params.max_convex_hull = max_convex_hull;
  params.preprocess_mode = preprocess_mode;
  params.prep_resolution = prep_resolution;
  params.resolution = sample_resolution;
  params.mcts_nodes = mcts_nodes;
  params.mcts_iteration = mcts_iteration;
  params.mcts_max_depth = mcts_max_depth;
  params.pca = pca;
  params.merge = merge;
  params.apx_mode = apx_mode;
  params.seed = seed;

  Model m;
  m.Load(input.vertices, input.indices);
  vector<double> bbox = m.Normalize();
  array<array<double, 3>, 3> rot{
      {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

  if (params.preprocess_mode == std::string("auto")) {
    bool is_manifold = IsManifold(m);
    logger::info("Mesh Manifoldness: {}", is_manifold);
    if (!is_manifold)
      ManifoldPreprocess(params, m);
  } else if (params.preprocess_mode == std::string("on")) {
    ManifoldPreprocess(params, m);
  }

  if (pca) {
    rot = m.PCA();
  }

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
  } else if (level == "debug") {
    logger::get()->set_level(spdlog::level::debug);
  } else if (level == "info") {
    logger::get()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    logger::get()->set_level(spdlog::level::warn);
  } else if (level == "error" || level == "err") {
    logger::get()->set_level(spdlog::level::err);
  } else if (level == "critical") {
    logger::get()->set_level(spdlog::level::critical);
  } else {
    throw std::runtime_error("invalid log level " + std::string(level));
  }
}

} // namespace coacd

extern "C" {
void CoACD_freeMeshArray(CoACD_MeshArray arr) {
  for (uint64_t i = 0; i < arr.meshes_count; ++i) {
    delete[] arr.meshes_ptr[i].vertices_ptr;
    arr.meshes_ptr[i].vertices_ptr = nullptr;
    arr.meshes_ptr[i].vertices_count = 0;
    delete[] arr.meshes_ptr[i].triangles_ptr;
    arr.meshes_ptr[i].triangles_ptr = nullptr;
    arr.meshes_ptr[i].triangles_count = 0;
  }
  arr.meshes_count = 0;
  arr.meshes_ptr = nullptr;
  delete[] arr.meshes_ptr;
}

CoACD_MeshArray CoACD_run(CoACD_Mesh const &input, double threshold,
                          int max_convex_hull, int preprocess_mode,
                          int prep_resolution, int sample_resolution,
                          int mcts_nodes, int mcts_iteration,
                          int mcts_max_depth, bool pca, bool merge,
                          int apx_mode, unsigned int seed) {
  coacd::Mesh mesh;
  for (uint64_t i = 0; i < input.vertices_count; ++i) {
    mesh.vertices.push_back({input.vertices_ptr[3 * i],
                             input.vertices_ptr[3 * i + 1],
                             input.vertices_ptr[3 * i + 2]});
  }
  for (uint64_t i = 0; i < input.triangles_count; ++i) {
    mesh.indices.push_back({input.triangles_ptr[3 * i],
                            input.triangles_ptr[3 * i + 1],
                            input.triangles_ptr[3 * i + 2]});
  }

  std::string pm, apx;
  if (preprocess_mode == preprocess_on) {
    pm = "on";
  } else if (preprocess_mode == preprocess_off) {
    pm = "off";
  } else {
    pm = "auto";
  }

  if (apx_mode == apx_ch) {
    apx = "ch";
  } else if (apx_mode == apx_box) {
    apx = "box";
  } else {
    throw std::runtime_error("invalid approximation mode " + std::to_string(apx_mode));
  }

  auto meshes = coacd::CoACD(mesh, threshold, max_convex_hull, pm,
                             prep_resolution, sample_resolution, mcts_nodes,
                             mcts_iteration, mcts_max_depth, pca, merge, apx, seed);

  CoACD_MeshArray arr;
  arr.meshes_ptr = new CoACD_Mesh[meshes.size()];
  arr.meshes_count = meshes.size();

  for (size_t i = 0; i < meshes.size(); ++i) {
    arr.meshes_ptr[i].vertices_ptr = new double[meshes[i].vertices.size() * 3];
    arr.meshes_ptr[i].vertices_count = meshes[i].vertices.size();
    for (size_t j = 0; j < meshes[i].vertices.size(); ++j) {
      arr.meshes_ptr[i].vertices_ptr[3 * j] = meshes[i].vertices[j][0];
      arr.meshes_ptr[i].vertices_ptr[3 * j + 1] = meshes[i].vertices[j][1];
      arr.meshes_ptr[i].vertices_ptr[3 * j + 2] = meshes[i].vertices[j][2];
    }
    arr.meshes_ptr[i].triangles_ptr = new int[meshes[i].indices.size() * 3];
    arr.meshes_ptr[i].triangles_count = meshes[i].indices.size();
    for (size_t j = 0; j < meshes[i].indices.size(); ++j) {
      arr.meshes_ptr[i].triangles_ptr[3 * j] = meshes[i].indices[j][0];
      arr.meshes_ptr[i].triangles_ptr[3 * j + 1] = meshes[i].indices[j][1];
      arr.meshes_ptr[i].triangles_ptr[3 * j + 2] = meshes[i].indices[j][2];
    }
  }
  return arr;
}

void CoACD_setLogLevel(char const *level) {
  coacd::set_log_level(std::string_view(level));
}
}
