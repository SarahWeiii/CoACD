#include "io.h"
#include "logger.h"

namespace coacd
{
    void SaveConfig(Params params)
    {
        logger::info(" - Config");
        logger::info("\tInput Path:                                {}", params.input_model);
        logger::info("\tOutput Path:                               {}", params.output_name);
        logger::info("\tRemesh Output Path:                        {}", params.remesh_output_name);
        logger::info("\tTerminal Threshold:                        {}", params.threshold);
        logger::info("\tMax # Convex Hulls:                        {}", params.max_convex_hull);
        logger::info("\tMCTS Node Number:                          {}", params.mcts_nodes);
        logger::info("\tMCTS Iteration:                            {}", params.mcts_iteration);
        logger::info("\tMCTS Max Depth:                            {}", params.mcts_max_depth);
        logger::info("\tManifold Preprocess Mode (auto/on/off):    {}", params.preprocess_mode);
        logger::info("\tPreprocess Resolution:                     {}", params.prep_resolution);
        logger::info("\tMerge Postprocess (on/off):                {}", params.merge);
        logger::info("\tPCA (ON/OFF):                              {}", params.pca);
        logger::info("\tk for Rv:                                  {}", params.rv_k);
        logger::info("\tHausdorff Sampling Resolution:             {}", params.resolution);
        logger::info("\tRandom Seed:                               {}", params.seed);
    }

    void SaveOBJ(const string &filename, vector<Model> parts, Params &params)
    {
        vector<int> v_numbers;
        v_numbers.push_back(0);
        std::ofstream os(filename);
        for (int n = 0; n < (int)parts.size(); n++)
        {
            os << "o convex_" << n << endl;
            for (int i = 0; i < (int)parts[n].points.size(); ++i)
            {
                os << "v " << parts[n].points[i][0] << " " << parts[n].points[i][1] << " " << parts[n].points[i][2] << "\n";
            }
            v_numbers.push_back(v_numbers[n] + (int)parts[n].points.size());
            for (int i = 0; i < (int)parts[n].triangles.size(); ++i)
            {
                os << "f " << parts[n].triangles[i][0] + 1 + v_numbers[n]
                   << " " << parts[n].triangles[i][1] + 1 + v_numbers[n]
                   << " " << parts[n].triangles[i][2] + 1 + v_numbers[n] << "\n";
            }
        }
        os.close();
    }

    void SaveOBJs(const string &foldername, const string &filename, vector<Model> parts, Params &params)
    {
        int n_zero = 3;
        for (int n = 0; n < (int)parts.size(); n++)
        {
            string num = to_string(n);
            string idx = string(n_zero - num.length(), '0') + num;
            ofstream os(foldername + "/" + filename + "_" + idx + ".obj");
            for (int i = 0; i < (int)parts[n].points.size(); ++i)
            {
                os << "v " << parts[n].points[i][0] << " " << parts[n].points[i][1] << " " << parts[n].points[i][2] << "\n";
            }
            for (int i = 0; i < (int)parts[n].triangles.size(); ++i)
            {
                os << "f " << parts[n].triangles[i][0] + 1
                   << " " << parts[n].triangles[i][1] + 1
                   << " " << parts[n].triangles[i][2] + 1 << "\n";
            }
            os.close();
        }
    }

    bool WriteVRML(ofstream &fout, Model mesh)
    {
        Material material;
        material.m_diffuseColor[0] = material.m_diffuseColor[1] = material.m_diffuseColor[2] = 0.0f;
        while (material.m_diffuseColor[0] == material.m_diffuseColor[1] || material.m_diffuseColor[2] == material.m_diffuseColor[1] || material.m_diffuseColor[2] == material.m_diffuseColor[0])
        {
            material.m_diffuseColor[0] = (rand() % 100) / 100.0f;
            material.m_diffuseColor[1] = (rand() % 100) / 100.0f;
            material.m_diffuseColor[2] = (rand() % 100) / 100.0f;
        }

        int nPoints = (int)mesh.points.size();
        int nTriangles = (int)mesh.triangles.size();
        if (fout.is_open())
        {
            fout.setf(ios::fixed, ios::floatfield);
            fout.setf(ios::showpoint);
            fout.precision(6);
            fout << "#VRML V2.0 utf8" << endl;
            fout << "" << endl;
            fout << "# Vertices: " << nPoints << endl;
            fout << "# Triangles: " << nTriangles << endl;
            fout << "" << endl;
            fout << "Group {" << endl;
            fout << "    children [" << endl;
            fout << "        Shape {" << endl;
            fout << "            appearance Appearance {" << endl;
            fout << "                material Material {" << endl;
            fout << "                    diffuseColor " << material.m_diffuseColor[0] << " "
                 << material.m_diffuseColor[1] << " "
                 << material.m_diffuseColor[2] << endl;
            fout << "                    ambientIntensity " << material.m_ambientIntensity << endl;
            fout << "                    specularColor " << material.m_specularColor[0] << " "
                 << material.m_specularColor[1] << " "
                 << material.m_specularColor[2] << endl;
            fout << "                    emissiveColor " << material.m_emissiveColor[0] << " "
                 << material.m_emissiveColor[1] << " "
                 << material.m_emissiveColor[2] << endl;
            fout << "                    shininess " << material.m_shininess << endl;
            fout << "                    transparency " << material.m_transparency << endl;
            fout << "                }" << endl;
            fout << "            }" << endl;
            fout << "            geometry IndexedFaceSet {" << endl;
            fout << "                ccw TRUE" << endl;
            fout << "                solid TRUE" << endl;
            fout << "                convex TRUE" << endl;
            if (nPoints > 0)
            {
                fout << "                coord DEF co Coordinate {" << endl;
                fout << "                    point [" << endl;
                for (int i = 0; i < nPoints; i++)
                    fout << "                        " << mesh.points[i][0] << " "
                         << mesh.points[i][1] << " "
                         << mesh.points[i][2] << "," << endl;
                fout << "                    ]" << endl;
                fout << "                }" << endl;
            }
            if (nTriangles > 0)
            {
                fout << "                coordIndex [ " << endl;
                for (int i = 0; i < nTriangles; i++)
                    fout << "                        " << mesh.triangles[i][0] << ", "
                         << mesh.triangles[i][1] << ", "
                         << mesh.triangles[i][2] << ", -1," << endl;
                fout << "                ]" << endl;
            }
            fout << "            }" << endl;
            fout << "        }" << endl;
            fout << "    ]" << endl;
            fout << "}" << endl;
            return true;
        }
        else
        {
            return false;
        }
    }

    void SaveVRML(const string &fileName, vector<Model>& meshes, Params &params)
    {
        ofstream foutCH(fileName);
        if (foutCH.is_open())
        {
            for (int p = 0; p < (int)meshes.size(); ++p)
            {
                WriteVRML(foutCH, meshes[p]);
            }
            foutCH.close();
        }
    }
}