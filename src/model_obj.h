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
#include "sobol.h"
#include "config.h"
#include "logger.h"

using std::map;

namespace coacd
{

    /* Copyright to V-HACD */
    struct Material
    {

        float m_diffuseColor[3];
        float m_ambientIntensity;
        float m_specularColor[3];
        float m_emissiveColor[3];
        float m_shininess;
        float m_transparency;
        Material(void)
        {
            m_diffuseColor[0] = 0.5f;
            m_diffuseColor[1] = 0.5f;
            m_diffuseColor[2] = 0.5f;
            m_specularColor[0] = 0.5f;
            m_specularColor[1] = 0.5f;
            m_specularColor[2] = 0.5f;
            m_ambientIntensity = 0.4f;
            m_emissiveColor[0] = 0.0f;
            m_emissiveColor[1] = 0.0f;
            m_emissiveColor[2] = 0.0f;
            m_shininess = 0.4f;
            m_transparency = 0.5f;
        };
    };

    class Model
    {
    public:
        double bbox[6];
        // double m_len, g_len;
        // double m_Xmid, m_Ymid, m_Zmid;
        double barycenter[3];
        array<array<double, 3>, 3> m_rot;

        vector<vec3d> points;
        vector<vec3i> triangles;

        Model();
        bool CheckThin();
        bool LoadOBJ(const string &fileName);
        bool Load(vector<vec3d> vertices, vector<vec3i> face_indices);
        void SaveOBJ(const string &fileName);
        array<array<double, 3>, 3> PCA();
        vector<double> Normalize();
        void Recover(vector<double> _bbox);
        void RevertPCA(array<array<double, 3>, 3> rot);
        void Clear();

        void GetEigenValues(array<array<double, 3>, 3> eigen_values);
        void AlignToPrincipalAxes();
        bool IsManifold();
        void ExtractPointSet(vector<vec3d> &samples, vector<int> &sample_tri_ids, unsigned int seed = 1235, size_t resolution = 2000, double base = 0, bool flag = false, Plane plane = Plane(0, 0, 0, 0));
        vector<vec3d> GetPoints(size_t resolution);
        double *GetBBox() { return bbox; }
        void ComputeCH(Model &convex);
        void ComputeVCH(Model &convex);
    };

    double MeshArea(Model &mesh);
    double MeshVolume(Model &mesh);
    void RecoverParts(vector<Model> &meshes, vector<double> bbox, array<array<double, 3>, 3> rot, Params &params);
    bool ComputeOverlapFace(Model &convex1, Model &convex2, Plane &plane);
    void ExtractPointSet(Model convex1, Model convex2, unsigned int seed, vector<vec3d> &samples, size_t resolution = 4000);
    void ExtractPointSet(Model &convex1, Model &convex2, vector<vec3d> &samples, vector<int> &sample_tri_ids, unsigned int seed, size_t resolution);
    void WritePointSet(const string &fileName, vector<vec3d> &samples);
    void MergeMesh(Model &mesh1, Model &mesh2, Model &merge);

}
