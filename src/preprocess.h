#pragma once
#include <openvdb/Exceptions.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/util/Util.h>
#include <vector>
#include <cstdio>
#include <string>
#include <omp.h>

#include "model_obj.h"
#include "logger.h"

using namespace openvdb;

namespace coacd
{
    void SDFManifold(Model &input, Model &output, bool if_cout, bool if_log, std::string logfile, double scale=10.0f);
}
