#pragma once
#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <assert.h>
#include <algorithm>
#include <set>
#include <map>
#include <unordered_map>

#include "shape.h"
#include "model_obj.h"

namespace coacd
{

    //////////////// IO ////////////////
    void SaveConfig(Params params);
    void SaveOBJ(const std::string &filename, std::vector<Model> parts, Params &params);
    void SaveOBJs(const std::string &foldername, const std::string &filename, std::vector<Model> parts, Params &params);
    bool WriteVRML(std::ofstream &fout, Model mesh);
    void SaveVRML(const std::string &fileName, std::vector<Model>& meshes, Params &params);
}