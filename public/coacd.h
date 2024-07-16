#pragma once
#include <array>
#include <string>
#include <string_view>
#include <vector>

namespace coacd {

#if _WIN32
#define COACD_API __declspec(dllexport)
#else
#define COACD_API
#endif

struct Mesh {
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<int, 3>> indices;
};

std::vector<Mesh> CoACD(Mesh const &input, double threshold = 0.05,
                        int max_convex_hull = -1, std::string preprocess = "auto",
                        int prep_resolution = 50, int sample_resolution = 2000,
                        int mcts_nodes = 20, int mcts_iteration = 150,
                        int mcts_max_depth = 3, bool pca = false,
                        bool merge = true, std::string apx_mode = "ch", unsigned int seed = 0);
void set_log_level(std::string_view level);

} // namespace coacd

extern "C" {

struct CoACD_Mesh {
  double *vertices_ptr;
  uint64_t vertices_count;
  int *triangles_ptr;
  uint64_t triangles_count;
};

struct CoACD_MeshArray {
  CoACD_Mesh *meshes_ptr;
  uint64_t meshes_count;
};

void COACD_API CoACD_freeMeshArray(CoACD_MeshArray arr);

constexpr int preprocess_auto = 0;
constexpr int preprocess_on = 1;
constexpr int preprocess_off = 2;

constexpr int apx_ch = 0;
constexpr int apx_box = 1;

CoACD_MeshArray COACD_API CoACD_run(CoACD_Mesh const &input, double threshold,
                                    int max_convex_hull, int preprocess_mode,
                                    int prep_resolution, int sample_resolution,
                                    int mcts_nodes, int mcts_iteration,
                                    int mcts_max_depth, bool pca, bool merge,
                                    bool decimate, int max_ch_vertex,
                                    int apx_mode, unsigned int seed);

void COACD_API CoACD_setLogLevel(char const *level);
}
