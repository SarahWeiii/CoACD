#pragma once
#include <array>
#include <string_view>
#include <vector>

namespace coacd {

struct Mesh {
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<int, 3>> indices;
};

std::vector<Mesh> CoACD(Mesh const &input, double threshold = 0.05, int max_convex_hull = -1,
                        bool preprocess = true, int prep_resolution = 50, int sample_resolution = 2000,
                        int mcts_nodes = 20, int mcts_iteration = 150, int mcts_max_depth = 3,
                        bool pca = false, bool merge = true, unsigned int seed = 1234);


void set_log_level(std::string_view level);

} // namespace coacd