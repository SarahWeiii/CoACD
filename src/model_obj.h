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
#include "glm/glm.hpp"
#include "manifold/types_p.h"

#include "shape.h"
#include "sobol.h"
#include "config.h"
#include "logger.h"

using namespace std;

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
        double m_len, g_len;
        double m_Xmid, m_Ymid, m_Zmid, g_Xmid, g_Ymid, g_Zmid;
        double barycenter[3];
        double m_rot[3][3];

        vector<vec3d> points;
        vector<vec3i> triangles;

        Model();
        bool CheckThin();
        bool LoadOBJ(const string &fileName);
        bool Load(vector<glm::dvec3> vertices, vector<glm::ivec3> face_indices);
        bool Load(MatrixD vertices, MatrixI face_indices);
        void SaveOBJ(const string &fileName);
        void PCA();
        void Normalize();
        void Recover();
        void RevertPCA();

        void GetEigenValues(double eigen_values[3][3]);
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
    void RecoverParts(vector<Model> &meshs, Params &params);
    bool ComputeOverlapFace(Model &convex1, Model &convex2, Plane &plane);
    void ExtractPointSet(Model convex1, Model convex2, unsigned int seed, vector<vec3d> &samples, size_t resolution = 4000);
    void ExtractPointSet(Model &convex1, Model &convex2, vector<vec3d> &samples, vector<int> &sample_tri_ids, unsigned int seed, size_t resolution);
    void WritePointSet(const string &fileName, vector<vec3d> &samples);
    void MergeMesh(Model &mesh1, Model &mesh2, Model &merge);

    inline void SyncNorm(const Model &mesh, Model &pCH)
    {
        pCH.g_len = mesh.g_len;
        pCH.g_Xmid = mesh.g_Xmid;
        pCH.g_Ymid = mesh.g_Ymid;
        pCH.g_Zmid = mesh.g_Zmid;
        pCH.m_len = mesh.m_len;
        pCH.m_Xmid = mesh.m_Xmid;
        pCH.m_Ymid = mesh.m_Ymid;
        pCH.m_Zmid = mesh.m_Zmid;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                pCH.m_rot[i][j] = mesh.m_rot[i][j];
    }

    inline void SyncNorm(const Model &mesh, Model &pos, Model &neg)
    {
        pos.g_len = neg.g_len = mesh.g_len;
        pos.g_Xmid = neg.g_Xmid = mesh.g_Xmid;
        pos.g_Ymid = neg.g_Ymid = mesh.g_Ymid;
        pos.g_Zmid = neg.g_Zmid = mesh.g_Zmid;
        pos.m_len = neg.m_len = mesh.m_len;
        pos.m_Xmid = neg.m_Xmid = mesh.m_Xmid;
        pos.m_Ymid = neg.m_Ymid = mesh.m_Ymid;
        pos.m_Zmid = neg.m_Zmid = mesh.m_Zmid;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                pos.m_rot[i][j] = neg.m_rot[i][j] = mesh.m_rot[i][j];
    }
}