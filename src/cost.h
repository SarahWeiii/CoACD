#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <math.h>
#include <limits>
#include <typeinfo>
#include <algorithm>
#include <assert.h>

#include "model_obj.h"
#pragma once

double ComputeRv(Model &tmesh1, Model &tmesh2, double k, double epsilon = 0.0001);
double ComputeRv(Model &cvx1, Model &cvx2, Model &cvxCH, double k, double epsilon = 0.0001);
double ComputeHb(Model &tmesh1, Model &tmesh2, unsigned int resolution, unsigned int seed, bool flag = false);
double ComputeHb(Model &cvx1, Model &cvx2, Model &cvxCH, unsigned int resolution, unsigned int seed);
double ComputeTotalRv(Model &mesh, Model &volume1, Model &volumeCH1, Model &volume2, Model &volumeCH2, double k, Plane &plane, double epsilon = 0.0001);
double ComputeHCost(Model &tmesh1, Model &tmesh2, double k, unsigned int resolution, unsigned int seed = 1235, double epsilon = 0.0001, bool flag = false);
double ComputeHCost(Model &cvx1, Model &cvx2, Model &cvxCH, double k, unsigned int resolution, unsigned int seed = 1235, double epsilon = 0.0001);
double ComputeEnergy(Model &mesh, Model &pos, Model &posCH, Model &neg, Model &negCH, double k, double cut_area, unsigned int resolution, unsigned int seed, double epsilon = 0.0001);
double MeshDist(Model &ch1, Model &ch2);
