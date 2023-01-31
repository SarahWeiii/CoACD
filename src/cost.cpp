#include "cost.h"
#include "hausdorff.h"

namespace coacd
{

  constexpr double Pi = 3.14159265;

  double ComputeRv(Model &tmesh1, Model &tmesh2, double k, double epsilon)
  {
    double v1, v2;
    v1 = MeshVolume(tmesh1);
    v2 = MeshVolume(tmesh2);

    double d = pow(3 * fabs(v1 - v2) / (4 * Pi), 1.0 / 3) * k;

    return d;
  }

  double ComputeRv(Model &cvx1, Model &cvx2, Model &cvxCH, double k, double epsilon)
  {
    double v1, v2, v3;

    v1 = MeshVolume(cvx1);
    v2 = MeshVolume(cvx2);
    v3 = MeshVolume(cvxCH);

    double d = pow(3 * fabs(v1 + v2 - v3) / (4 * Pi), 1.0 / 3) * k;

    return d;
  }

  double ComputeHb(Model &tmesh1, Model &tmesh2, unsigned int resolution, unsigned int seed, bool flag)
  {
    vector<vec3d> samples1, samples2;
    vector<int> sample_tri_ids1, sample_tri_ids2;

    tmesh1.ExtractPointSet(samples1, sample_tri_ids1, seed, resolution, 1);
    tmesh2.ExtractPointSet(samples2, sample_tri_ids2, seed, resolution, 1);

    if (!((int)samples1.size() > 0 && (int)samples2.size() > 0))
      return INF;

    double h;
    h = face_hausdorff_distance(tmesh1, samples1, sample_tri_ids1, tmesh2, samples2, sample_tri_ids2);

    return h;
  }
  double ComputeHb(Model &cvx1, Model &cvx2, Model &cvxCH, unsigned int resolution, unsigned int seed)
  {
    if (cvx1.points.size() + cvx2.points.size() == cvxCH.points.size())
      return 0.0;
    Model cvx;
    vector<vec3d> samples1, samples2;
    vector<int> sample_tri_ids1, sample_tri_ids2;
    MergeMesh(cvx1, cvx2, cvx);
    ExtractPointSet(cvx1, cvx2, samples1, sample_tri_ids1, seed, resolution);
    cvxCH.ExtractPointSet(samples2, sample_tri_ids2, seed, resolution, 1);

    if (!((int)samples1.size() > 0 && (int)samples2.size() > 0))
      return INF;

    double h = face_hausdorff_distance(cvx, samples1, sample_tri_ids1, cvxCH, samples2, sample_tri_ids2);

    return h;
  }

  double ComputeTotalRv(Model &mesh, Model &volume1, Model &volumeCH1, Model &volume2, Model &volumeCH2, double k, Plane &plane, double epsilon)
  {
    double h_pos = ComputeRv(volume1, volumeCH1, k, epsilon);
    double h_neg = ComputeRv(volume2, volumeCH2, k, epsilon);

    return max(h_pos, h_neg);
  }

  double ComputeHCost(Model &tmesh1, Model &tmesh2, double k, unsigned int resolution, unsigned int seed, double epsilon, bool flag)
  {
    double h1 = ComputeRv(tmesh1, tmesh2, k, epsilon);
    double h2 = ComputeHb(tmesh1, tmesh2, resolution, seed, flag);

    return max(h1, h2);
  }

  double ComputeHCost(Model &cvx1, Model &cvx2, Model &cvxCH, double k, unsigned int resolution, unsigned int seed, double epsilon)
  {
    double h1 = ComputeRv(cvx1, cvx2, cvxCH, k, epsilon);
    double h2 = ComputeHb(cvx1, cvx2, cvxCH, resolution + 2000, seed);

    return max(h1, h2);
  }

  double ComputeEnergy(Model &mesh, Model &pos, Model &posCH, Model &neg, Model &negCH, double k, double cut_area, unsigned int resolution, unsigned int seed, double epsilon)
  {
    double h_pos = ComputeHCost(pos, posCH, k, resolution, seed, epsilon);
    double h_neg = ComputeHCost(neg, negCH, k, resolution, seed, epsilon);
    return max(h_pos, h_neg);
  }

  double MeshDist(Model &ch1, Model &ch2)
  {
    vector<vec3d> XA = ch1.points, XB = ch2.points;

    int nA = XA.size();

    PointCloud<double> cloudB;
    vec2PointCloud(cloudB, XB);

    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>,
        PointCloud<double>,
        3 /* dim */
        >
        my_kd_tree_t;

    my_kd_tree_t indexB(3 /*dim*/, cloudB, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    indexB.buildIndex();

    double minDist = INF;
    for (int i = 0; i < nA; i++)
    {
      size_t num_results = 1;

      double query_pt[3] = {XA[i][0], XA[i][1], XA[i][2]};

      std::vector<size_t> ret_index(num_results);
      std::vector<double> out_dist_sqr(num_results);

      num_results = indexB.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
      double dist = sqrt(out_dist_sqr[0]);
      minDist = min(minDist, dist);
    }

    return minDist;
  }
}