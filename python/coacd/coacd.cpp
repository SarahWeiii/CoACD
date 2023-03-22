#include <pybind11/pybind11.h>

#include "coacd.hpp"

namespace py = pybind11;

PYBIND11_MODULE(coacd, m) {
  m.doc() = "Collision-aware ACD python binding";
  build_coacd(m);
}