#pragma once
#include <openvdb/Exceptions.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/util/Util.h>
#include <vector>
#include <cstdio>
#include <string>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <algorithm>

#include "model_obj.h"
#include "logger.h"
#include "bvh.h"

using namespace openvdb;

namespace coacd
{

    void SDFManifold(Model &input, Model &output, double scale = 50.0f, double level_set = 0.55f);
    bool IsManifold(Model &input);
}
