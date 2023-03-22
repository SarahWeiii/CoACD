#pragma once
#include <array>
#include <string_view>
#include <vector>

namespace coacd {

struct Mesh {
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<int, 3>> indices;
};

std::vector<Mesh> CoACD(Mesh const &input, int downsampling = 20, double threshold = 0.05,
                        int max_nConvexHull = -1, int resolution = 2000, unsigned int seed = 1234,
                        bool preprocess = true, int prep_resolution = 30, 
                        bool pca = false, bool merge = true,
                        int mcts_iteration = 150, int mcts_max_depth = 3);

Mesh Remesh(Mesh const &input, int resolution = 30, double level_set = 0.55);

void set_log_level(std::string_view level);

} // namespace coacd