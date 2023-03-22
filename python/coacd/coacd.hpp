#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../../public/coacd.h"

namespace py = pybind11;

void build_coacd(py::module &m)
{
    auto PyMesh = py::class_<coacd::Mesh>(m, "Mesh");
    PyMesh.def(py::init<>());
    PyMesh.def_readwrite("vertices", &coacd::Mesh::vertices);
    PyMesh.def_readwrite("indices", &coacd::Mesh::indices);

    m.def("run_coacd", &coacd::CoACD, py::arg("mesh"), py::arg("mcts_nodes") = 20, py::arg("threshold") = 0.05,
            py::arg("max_convex_hull") = -1, py::arg("resolution") = 2000, py::arg("seed") = 0, py::arg("preprocess") = true,
            py::arg("preprocess_resolution") = 30, py::arg("pca") = false, py::arg("merge") = true, py::arg("mcts_iterations") = 150, py::arg("mcts_max_depth") = 3);
}