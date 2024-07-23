#pragma once

#include <iostream>
#include <string>
#include <random>
#include <fstream>
#include <vector>
#include <math.h>
#include <limits>
#include <typeinfo>
#include <algorithm>
#include <assert.h>
#include <regex>

#include "./io.h"
#include "config.h"
#include "model_obj.h"
#include "shape.h"

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

namespace coacd
{
    // Sphere welzl(std::vector<vec3d> &P, int num_runs = 5);
    Sphere welzl(Model &mesh, int num_runs = 5);
    void Sphere2Mesh(Sphere &sphere, Model &mesh, int resolution = 20);
}