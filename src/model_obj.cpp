#include <stdint.h>
#include "model_obj.h"
#include "quickhull/QuickHull.hpp"
#include "btConvexHull/btConvexHullComputer.h"
#include "nanoflann.hpp"

using namespace nanoflann;

namespace coacd 
{

Model::Model()
{
    barycenter[0] = 0.0;
    barycenter[1] = 0.0;
    barycenter[2] = 0.0;
}

bool Model::CheckThin()
{
    Normalize();
    int idx0 = 0;
    int idx1;
    int idx2;
    bool flag = 0;

    for (int i = 1; i < (int)points.size(); i++)
    {
        double dist = sqrt(pow(points[idx0][0] - points[i][0], 2) + pow(points[idx0][1] - points[i][1], 2) + pow(points[idx0][2] - points[i][2], 2));
        if (dist > 0.01)
        {
            flag = 1;
            idx1 = i;
            break;
        }
    }
    if (!flag)
        return true;
    flag = 0;

    for (int i = 2; i < (int)points.size(); i++)
    {
        if (i == idx1)
            continue;
        vec3d p0 = points[idx0];
        vec3d p1 = points[idx1];
        vec3d p2 = points[i];
        vec3d AB, BC;
        AB[0] = p1[0] - p0[0];
        AB[1] = p1[1] - p0[1];
        AB[2] = p1[2] - p0[2];
        BC[0] = p2[0] - p1[0];
        BC[1] = p2[1] - p1[1];
        BC[2] = p2[2] - p1[2];

        double dot_product = AB[0] * BC[0] + AB[1] * BC[1] + AB[2] * BC[2];
        double res = dot_product / (sqrt(pow(AB[0], 2) + pow(AB[1], 2) + pow(AB[2], 2)) * sqrt(pow(BC[0], 2) + pow(BC[1], 2) + pow(BC[2], 2)));
        if (fabs(fabs(res) - 1) > 1e-6 && fabs(res) < INF) // AB not \\ BC, dot product != 1
        {
            flag = 1;
            idx2 = i;
            break;
        }
    }
    if (!flag)
        return true;

    vec3d p0 = points[idx0], p1 = points[idx1], p2 = points[idx2];

    Plane p;
    double a = (p1[1] - p0[1]) * (p2[2] - p0[2]) - (p1[2] - p0[2]) * (p2[1] - p0[1]);
    double b = (p1[2] - p0[2]) * (p2[0] - p0[0]) - (p1[0] - p0[0]) * (p2[2] - p0[2]);
    double c = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    p.a = a / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
    p.b = b / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
    p.c = c / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
    p.d = 0 - (p.a * p1[0] + p.b * p1[1] + p.c * p1[2]);

    for (int i = 0; i < (int)points.size(); i++)
    {
        if (p.Side(points[i]) != 0)
            return false;
    }
    return true;

}

vector<vec3d> Model::GetPoints(size_t resolution)
{
    if (resolution > (size_t)points.size())
        return points;
    else
    {
        vector<vec3d> tmp;
        for (int i = 0; i < (int)points.size(); i++)
        {
            srand(i * (unsigned)time(NULL));
            if (rand() / double(RAND_MAX) < (double(resolution) / (double)points.size()))
                tmp.push_back(points[i]);
        }
        return tmp;
    }
}

void Model::ComputeCH(Model &convex)
{
    /* fast convex hull algorithm */
    bool flag = true;
    quickhull::QuickHull<float> qh; // Could be double as well
    vector<quickhull::Vector3<float>> pointCloud;
    // Add points to point cloud
    for (int i = 0; i < (int)points.size(); i++)
    {
        pointCloud.push_back(quickhull::Vector3<float>(points[i][0], points[i][1], points[i][2]));
    }

    auto hull = qh.getConvexHull(pointCloud, true, false, flag);
    if (!flag)
    {
        // backup convex hull algorithm, stable but slow
        ComputeVCH(convex);
        return;
    }
    const auto &indexBuffer = hull.getIndexBuffer();
    const auto &vertexBuffer = hull.getVertexBuffer();
    for (int i = 0; i < (int)vertexBuffer.size(); i++)
    {
        convex.points.push_back({vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z});
    }
    for (int i = 0; i < (int)indexBuffer.size(); i += 3)
    {
        convex.triangles.push_back({(int)indexBuffer[i + 2], (int)indexBuffer[i + 1], (int)indexBuffer[i]});
    }
}

void Model::ComputeVCH(Model &convex)
{
    btConvexHullComputer ch;
    ch.compute(points, -1.0, -1.0);
    for (int32_t v = 0; v < ch.vertices.size(); v++)
    {
        convex.points.push_back({ch.vertices[v].getX(), ch.vertices[v].getY(), ch.vertices[v].getZ()});
    }
    const int32_t nt = ch.faces.size();
    for (int32_t t = 0; t < nt; ++t)
    {
        const btConvexHullComputer::Edge *sourceEdge = &(ch.edges[ch.faces[t]]);
        int32_t a = sourceEdge->getSourceVertex();
        int32_t b = sourceEdge->getTargetVertex();
        const btConvexHullComputer::Edge *edge = sourceEdge->getNextEdgeOfFace();
        int32_t c = edge->getTargetVertex();
        while (c != a)
        {
            convex.triangles.push_back({(int)a, (int)b, (int)c});
            edge = edge->getNextEdgeOfFace();
            b = c;
            c = edge->getTargetVertex();
        }
    }
}

void Model::GetEigenValues(double eigen_values[3][3])
{
    double Q[3][3];
    double barycenter[3] = {0};
    for (int i = 0; i < (int)points.size(); i++)
    {
        barycenter[0] += points[i][0];
        barycenter[1] += points[i][1];
        barycenter[2] += points[i][2];
    }
    barycenter[0] /= (int)points.size();
    barycenter[1] /= (int)points.size();
    barycenter[2] /= (int)points.size();

    double covMat[3][3] = {{0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0}};
    double x, y, z;
    for (int i = 0; i < (int)points.size(); i++)
    {
        x = points[i][0] - barycenter[0];
        y = points[i][1] - barycenter[1];
        z = points[i][2] - barycenter[2];
        covMat[0][0] += x * x;
        covMat[1][1] += y * y;
        covMat[2][2] += z * z;
        covMat[0][1] += x * y;
        covMat[0][2] += x * z;
        covMat[1][2] += y * z;
    }
    covMat[0][0] /= (int)points.size();
    covMat[1][1] /= (int)points.size();
    covMat[2][2] /= (int)points.size();
    covMat[0][1] /= (int)points.size();
    covMat[0][2] /= (int)points.size();
    covMat[1][2] /= (int)points.size();
    covMat[1][0] = covMat[0][1];
    covMat[2][0] = covMat[0][2];
    covMat[2][1] = covMat[1][2];
    Diagonalize(covMat, Q, eigen_values);
}

void Model::AlignToPrincipalAxes()
{
    for (int i = 0; i < (int)points.size(); i++)
    {
        barycenter[0] += points[i][0];
        barycenter[1] += points[i][1];
        barycenter[2] += points[i][2];
    }
    barycenter[0] /= (int)points.size();
    barycenter[1] /= (int)points.size();
    barycenter[2] /= (int)points.size();

    double covMat[3][3] = {{0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0}};
    double x, y, z;
    for (int i = 0; i < (int)points.size(); i++)
    {
        x = points[i][0] - barycenter[0];
        y = points[i][1] - barycenter[1];
        z = points[i][2] - barycenter[2];
        covMat[0][0] += x * x;
        covMat[1][1] += y * y;
        covMat[2][2] += z * z;
        covMat[0][1] += x * y;
        covMat[0][2] += x * z;
        covMat[1][2] += y * z;
    }
    covMat[0][0] /= (int)points.size();
    covMat[1][1] /= (int)points.size();
    covMat[2][2] /= (int)points.size();
    covMat[0][1] /= (int)points.size();
    covMat[0][2] /= (int)points.size();
    covMat[1][2] /= (int)points.size();
    covMat[1][0] = covMat[0][1];
    covMat[2][0] = covMat[0][2];
    covMat[2][1] = covMat[1][2];
    double D[3][3];
    Diagonalize(covMat, m_rot, D);
}

inline void addEdge(map<pair<int, int>, bool> &edge_map, int id1, int id2)
{
    pair<int, int> edge1 = make_pair(id1, id2);
    pair<int, int> edge2 = make_pair(id2, id1);
    if (edge_map.find(edge1) == edge_map.end() && edge_map.find(edge2) == edge_map.end())
        edge_map[edge1] = true;
}

bool ComputeOverlapFace(Model &convex1, Model &convex2, Plane &plane)
{
    bool flag;
    for (int i = 0; i < (int)convex1.triangles.size(); i++)
    {
        Plane p;
        vec3d p1, p2, p3;
        p1 = convex1.points[convex1.triangles[i][0]];
        p2 = convex1.points[convex1.triangles[i][1]];
        p3 = convex1.points[convex1.triangles[i][2]];
        double a = (p2[1] - p1[1]) * (p3[2] - p1[2]) - (p2[2] - p1[2]) * (p3[1] - p1[1]);
        double b = (p2[2] - p1[2]) * (p3[0] - p1[0]) - (p2[0] - p1[0]) * (p3[2] - p1[2]);
        double c = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]);
        p.a = a / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
        p.b = b / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
        p.c = c / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
        p.d = 0 - (p.a * p1[0] + p.b * p1[1] + p.c * p1[2]);

        short side1 = 0;
        for (int j = 0; j < (int)convex1.points.size(); j++)
        {
            short s = p.Side(convex1.points[j], 1e-8);
            if (s != 0)
            {
                side1 = s;
                flag = 1;
                break;
            }
        }

        for (int j = 0; j < (int)convex2.points.size(); j++)
        {
            short s = p.Side(convex2.points[j], 1e-8);
            if (!flag || s == side1)
            {
                flag = 0;
                break;
            }
        }
        if (flag)
        {
            plane = p;
            return true;
        }
    }
    return false;
}

void WritePointSet(const string &fileName, vector<vec3d> &samples)
{
    std::ofstream os(fileName);
    for (int i = 0; i < (int)samples.size(); i++)
    {
        os << "v " << samples[i][0] << ' ' << samples[i][1] << ' ' << samples[i][2] << endl;
    }
    os.close();
}

void Model::ExtractPointSet(vector<vec3d> &samples, vector<int> &sample_tri_ids, unsigned int seed, size_t resolution, double base, bool flag, Plane plane)
{
    if (resolution == 0)
        return;
    double aObj = 0;
    for (int i = 0; i < (int)triangles.size(); i++)
    {
        aObj += Area(points[triangles[i][0]], points[triangles[i][1]], points[triangles[i][2]]);
    }

    if (base != 0)
        resolution = size_t(max(1000, int(resolution * (aObj / base))));

    srand(seed);
    for (int i = 0; i < (int)triangles.size(); i++)
    {
        if (flag && plane.Side(points[triangles[i][0]], 1e-3) == 0 &&
            plane.Side(points[triangles[i][1]], 1e-3) == 0 &&
            plane.Side(points[triangles[i][2]], 1e-3) == 0)
        {
            continue;
        }
        double area = Area(points[triangles[i][0]], points[triangles[i][1]], points[triangles[i][2]]);
        int N;
        if ((size_t)triangles.size() > resolution && resolution)
            N = max(int(i % ((int)triangles.size() / resolution) == 0), int(resolution / aObj * area));
        else
            N = max(int(i % 2 == 0), int(resolution / aObj * area));

        int seed = rand() % 1000;
        float r[2];
        for (int k = 0; k < N; k++)
        {
            double a, b;
            if (k % 3 == 0)
            {
                //// random sample
                a = rand() / double(RAND_MAX);
                b = rand() / double(RAND_MAX);
            }
            else
            {
                //// quasirandom sample
                i4_sobol(2, &seed, r);
                a = r[0];
                b = r[1];
            }

            vec3d v;
            v[0] = (1 - sqrt(a)) * points[triangles[i][0]][0] + (sqrt(a) * (1 - b)) * points[triangles[i][1]][0] + b * sqrt(a) * points[triangles[i][2]][0];
            v[1] = (1 - sqrt(a)) * points[triangles[i][0]][1] + (sqrt(a) * (1 - b)) * points[triangles[i][1]][1] + b * sqrt(a) * points[triangles[i][2]][1];
            v[2] = (1 - sqrt(a)) * points[triangles[i][0]][2] + (sqrt(a) * (1 - b)) * points[triangles[i][1]][2] + b * sqrt(a) * points[triangles[i][2]][2];
            samples.push_back(v);
            sample_tri_ids.push_back(i);
        }
    }
}

void ExtractPointSet(Model convex1, Model convex2, unsigned int seed, vector<vec3d> &samples, size_t resolution)
{
    vector<vec3d> XA, XB;
    double a1 = 0, a2 = 0;
    for (int i = 0; i < (int)convex1.triangles.size(); i++)
        a1 += Area(convex1.points[convex1.triangles[i][0]], convex1.points[convex1.triangles[i][1]], convex1.points[convex1.triangles[i][2]]);
    for (int i = 0; i < (int)convex2.triangles.size(); i++)
        a2 += Area(convex2.points[convex2.triangles[i][0]], convex2.points[convex2.triangles[i][1]], convex2.points[convex2.triangles[i][2]]);

    Plane overlap_plane;
    bool flag = ComputeOverlapFace(convex1, convex2, overlap_plane);

    vector<int> tmp1, tmp2;
    convex1.ExtractPointSet(XA, tmp1, seed, size_t(a1 / (a1 + a2) * resolution), 1, flag, overlap_plane);
    convex2.ExtractPointSet(XB, tmp2, seed, size_t(a2 / (a1 + a2) * resolution), 1, flag, overlap_plane);

    samples.insert(samples.end(), XA.begin(), XA.end());
    samples.insert(samples.end(), XB.begin(), XB.end());
}

void ExtractPointSet(Model &convex1, Model &convex2, vector<vec3d> &samples, vector<int> &sample_tri_ids, unsigned int seed, size_t resolution)
{
    vector<vec3d> samples1, samples2;
    vector<int> sample_tri_ids1, sample_tri_ids2;
    double a1 = 0, a2 = 0;
    for (int i = 0; i < (int)convex1.triangles.size(); i++)
        a1 += Area(convex1.points[convex1.triangles[i][0]], convex1.points[convex1.triangles[i][1]], convex1.points[convex1.triangles[i][2]]);
    for (int i = 0; i < (int)convex2.triangles.size(); i++)
        a2 += Area(convex2.points[convex2.triangles[i][0]], convex2.points[convex2.triangles[i][1]], convex2.points[convex2.triangles[i][2]]);

    Plane overlap_plane;
    bool flag = ComputeOverlapFace(convex1, convex2, overlap_plane);

    convex1.ExtractPointSet(samples1, sample_tri_ids1, seed, size_t(a1 / (a1 + a2) * resolution), 1, flag, overlap_plane);
    convex2.ExtractPointSet(samples2, sample_tri_ids2, seed, size_t(a2 / (a1 + a2) * resolution), 1, flag, overlap_plane);

    samples.insert(samples.end(), samples1.begin(), samples1.end());
    samples.insert(samples.end(), samples2.begin(), samples2.end());

    sample_tri_ids.insert(sample_tri_ids.end(), sample_tri_ids1.begin(), sample_tri_ids1.end());
    int N = (int)convex1.triangles.size();
    for (int i = 0; i < (int)sample_tri_ids2.size(); i++)
        sample_tri_ids.push_back(sample_tri_ids2[i] + N);
}

void MergeMesh(Model &mesh1, Model &mesh2, Model &merge)
{
    merge.points.insert(merge.points.end(), mesh1.points.begin(), mesh1.points.end());
    merge.points.insert(merge.points.end(), mesh2.points.begin(), mesh2.points.end());
    merge.triangles.insert(merge.triangles.end(), mesh1.triangles.begin(), mesh1.triangles.end());
    int N = mesh1.points.size();
    for (int i = 0; i < (int)mesh2.triangles.size(); i++)
        merge.triangles.push_back({mesh2.triangles[i][0] + N, mesh2.triangles[i][1] + N, mesh2.triangles[i][2] + N});
}

/////////// IO /////////////

bool Model::LoadOBJ(const string &fileName)
{
    const unsigned int BufferSize = 1024;
    FILE *fid = fopen(fileName.c_str(), "r");

    if (fid)
    {
        char buffer[BufferSize];
        int ip[4];
        double x[3];
        char *pch;
        char *str;
        double x_min = INF, x_max = -INF, y_min = INF, y_max = -INF, z_min = INF, z_max = -INF;
        while (!feof(fid))
        {
            if (!fgets(buffer, BufferSize, fid))
            {
                break;
            }
            else if (buffer[0] == 'v')
            {
                if (buffer[1] == ' ')
                {
                    str = buffer + 2;
                    for (int k = 0; k < 3; ++k)
                    {
                        pch = strtok(str, " ");
                        if (pch)
                            x[k] = (double)atof(pch);
                        else
                        {
                            return false;
                        }
                        str = NULL;
                    }
                    points.push_back({x[0], x[1], x[2]});

                    x_min = min(x_min, x[0]);
                    x_max = max(x_max, x[0]);
                    y_min = min(y_min, x[1]);
                    y_max = max(y_max, x[1]);
                    z_min = min(z_min, x[2]);
                    z_max = max(z_max, x[2]);
                }
            }
            else if (buffer[0] == 'f')
            {

                pch = str = buffer + 2;
                int k = 0;
                while (pch)
                {
                    pch = strtok(str, " ");
                    if (pch && *pch != '\n')
                    {
                        ip[k++] = (unsigned int)atoi(pch) - 1;
                    }
                    else
                    {
                        break;
                    }
                    str = NULL;
                }
                if (k == 3)
                {
                    triangles.push_back({ip[0], ip[1], ip[2]});
                }
                else if (k == 4)
                {
                    triangles.push_back({ip[0], ip[1], ip[2]});
                    triangles.push_back({ip[0], ip[2], ip[3]});
                }
            }
        }

        m_len = max(max(x_max - x_min, y_max - y_min), z_max - z_min);
        m_Xmid = (x_max + x_min) / 2;
        m_Ymid = (y_max + y_min) / 2;
        m_Zmid = (z_max + z_min) / 2;
        bbox[0] = x_min;
        bbox[1] = x_max;
        bbox[2] = y_min;
        bbox[3] = y_max;
        bbox[4] = z_min;
        bbox[5] = z_max;

        fclose(fid);
    }
    else
    {
        cout << "Open File Error!" << endl;
        return false;
    }
    return true;
}

bool Model::Load(vector<glm::dvec3> vertices, vector<glm::ivec3> face_indices)
{
    double x_min = INF, x_max = -INF, y_min = INF, y_max = -INF, z_min = INF, z_max = -INF;
    for (int i = 0; i < (int)vertices.size(); ++i)
    {
        points.push_back({vertices[i][0], vertices[i][1], vertices[i][2]});

        x_min = min(x_min, vertices[i][0]);
        x_max = max(x_max, vertices[i][0]);
        y_min = min(y_min, vertices[i][1]);
        y_max = max(y_max, vertices[i][1]);
        z_min = min(z_min, vertices[i][2]);
        z_max = max(z_max, vertices[i][2]);
    }

    m_len = max(max(x_max - x_min, y_max - y_min), z_max - z_min);
    m_Xmid = (x_max + x_min) / 2;
    m_Ymid = (y_max + y_min) / 2;
    m_Zmid = (z_max + z_min) / 2;
    bbox[0] = x_min;
    bbox[1] = x_max;
    bbox[2] = y_min;
    bbox[3] = y_max;
    bbox[4] = z_min;
    bbox[5] = z_max;

    for (int i = 0; i < (int)face_indices.size(); ++i)
    {
        triangles.push_back({face_indices[i][0], face_indices[i][1], face_indices[i][2]});
    }

    return true;
}

bool Model::Load(MatrixD vertices, MatrixI face_indices)
{
    double x_min = INF, x_max = -INF, y_min = INF, y_max = -INF, z_min = INF, z_max = -INF;
    for (int i = 0; i < (int)vertices.rows(); ++i)
    {
        points.push_back({vertices(i, 0), vertices(i, 1), vertices(i, 2)});

        x_min = min(x_min, vertices(i, 0));
        x_max = max(x_max, vertices(i, 0));
        y_min = min(y_min, vertices(i, 1));
        y_max = max(y_max, vertices(i, 1));
        z_min = min(z_min, vertices(i, 2));
        z_max = max(z_max, vertices(i, 2));
    }

    m_len = max(max(x_max - x_min, y_max - y_min), z_max - z_min);
    m_Xmid = (x_max + x_min) / 2;
    m_Ymid = (y_max + y_min) / 2;
    m_Zmid = (z_max + z_min) / 2;
    bbox[0] = x_min;
    bbox[1] = x_max;
    bbox[2] = y_min;
    bbox[3] = y_max;
    bbox[4] = z_min;
    bbox[5] = z_max;

    for (int i = 0; i < (int)face_indices.rows(); ++i)
    {
        triangles.push_back({face_indices(i, 0), face_indices(i, 1), face_indices(i, 2)});
    }

    return true;
}

void Model::PCA()
{
    AlignToPrincipalAxes();
    double x_min = INF, x_max = -INF, y_min = INF, y_max = -INF, z_min = INF, z_max = -INF;
    for (int i = 0; i < (int)points.size(); i++)
    {
        double x = points[i][0];
        double y = points[i][1];
        double z = points[i][2];
        points[i][0] = m_rot[0][0] * x + m_rot[1][0] * y + m_rot[2][0] * z;
        points[i][1] = m_rot[0][1] * x + m_rot[1][1] * y + m_rot[2][1] * z;
        points[i][2] = m_rot[0][2] * x + m_rot[1][2] * y + m_rot[2][2] * z;

        x_min = min(x_min, points[i][0]);
        x_max = max(x_max, points[i][0]);
        y_min = min(y_min, points[i][1]);
        y_max = max(y_max, points[i][1]);
        z_min = min(z_min, points[i][2]);
        z_max = max(z_max, points[i][2]);
    }

    m_len = max(max(x_max - x_min, y_max - y_min), z_max - z_min);
    m_Xmid = (x_max + x_min) / 2;
    m_Ymid = (y_max + y_min) / 2;
    m_Zmid = (z_max + z_min) / 2;
    bbox[0] = x_min;
    bbox[1] = x_max;
    bbox[2] = y_min;
    bbox[3] = y_max;
    bbox[4] = z_min;
    bbox[5] = z_max;
}

void Model::Normalize()
{
    for (int i = 0; i < (int)points.size(); i++)
    {
        vec3d tmp = {2.0 * (points[i][0] - m_Xmid) / m_len,
                     2.0 * (points[i][1] - m_Ymid) / m_len,
                     2.0 * (points[i][2] - m_Zmid) / m_len};
        points[i] = tmp;
    }
    double x_len = bbox[1] - bbox[0], y_len = bbox[3] - bbox[2], z_len = bbox[5] - bbox[4];
    bbox[0] = -x_len / m_len;
    bbox[1] = x_len / m_len;
    bbox[2] = -y_len / m_len;
    bbox[3] = y_len / m_len;
    bbox[4] = -z_len / m_len;
    bbox[5] = z_len / m_len;
}

void Model::Recover()
{
    for (int i = 0; i < (int)points.size(); i++)
        points[i] = {points[i][0] / 2 * m_len + m_Xmid,
                     points[i][1] / 2 * m_len + m_Ymid,
                     points[i][2] / 2 * m_len + m_Zmid};
}

void Model::RevertPCA()
{
    for (int i = 0; i < (int)points.size(); i++)
    {
        double x = points[i][0];
        double y = points[i][1];
        double z = points[i][2];
        points[i][0] = m_rot[0][0] * x + m_rot[0][1] * y + m_rot[0][2] * z;
        points[i][1] = m_rot[1][0] * x + m_rot[1][1] * y + m_rot[1][2] * z;
        points[i][2] = m_rot[2][0] * x + m_rot[2][1] * y + m_rot[2][2] * z;
    }
}

void Model::SaveOBJ(const string &filename)
{
    std::ofstream os(filename);
    for (int i = 0; i < (int)points.size(); ++i)
    {
        os << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << "\n";
    }
    for (int i = 0; i < (int)triangles.size(); ++i)
    {
        os << "f " << triangles[i][0] + 1 << " " << triangles[i][1] + 1 << " " << triangles[i][2] + 1 << "\n";
    }
    os.close();
}

double MeshArea(Model &mesh)
{
    double area = 0;
    for (int i = 0; i < (int)mesh.triangles.size(); i++)
    {
        int idx0 = mesh.triangles[i][0], idx1 = mesh.triangles[i][1], idx2 = mesh.triangles[i][2];
        area += Area(mesh.points[idx0], mesh.points[idx1], mesh.points[idx2]);
    }
    return area;
}

double MeshVolume(Model &mesh)
{
    double volume = 0;
    for (int i = 0; i < (int)mesh.triangles.size(); i++)
    {
        int idx0 = mesh.triangles[i][0], idx1 = mesh.triangles[i][1], idx2 = mesh.triangles[i][2];
        volume += Volume(mesh.points[idx0], mesh.points[idx1], mesh.points[idx2]);
    }
    return volume;
}

void RecoverParts(vector<Model> &meshs, Params &params)
{
    for (int i = 0; i < (int)meshs.size(); i++)
    {
        meshs[i].Recover();
        if (params.pca)
            meshs[i].RevertPCA();
    }
}
}