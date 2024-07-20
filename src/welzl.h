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

    // struct vec3d {
    //     double x, y, z;
    //     vec3d(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    // };

    Sphere welzl(std::vector<vec3d> &P, int num_runs = 3);

}