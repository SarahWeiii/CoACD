#include "welzl.h"
namespace coacd
{

    void Sphere2Mesh(Sphere &sphere, Model &mesh, int resolution)
    {
        mesh.Clear();

        // Generate vertices
        for (int i = 0; i <= resolution; ++i)
        {
            double phi = M_PI * i / resolution;
            for (int j = 0; j <= resolution; ++j)
            {
                double theta = 2 * M_PI * j / resolution;
                double x = sphere.center[0] + sphere.radius * sin(phi) * cos(theta);
                double y = sphere.center[1] + sphere.radius * sin(phi) * sin(theta);
                double z = sphere.center[2] + sphere.radius * cos(phi);
                mesh.points.push_back({x, y, z});
            }
        }

        // Generate faces
        for (int i = 0; i < resolution; ++i)
        {
            for (int j = 0; j < resolution; ++j)
            {
                int p1 = i * (resolution + 1) + j;
                int p2 = p1 + 1;
                int p3 = (i + 1) * (resolution + 1) + j;
                int p4 = p3 + 1;

                mesh.triangles.push_back({p3, p2, p1});
                mesh.triangles.push_back({p3, p4, p2});
            }
        }
    }

    double distance(const vec3d &a, const vec3d &b)
    {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    vec3d barycenter(const std::vector<vec3d> &points)
    {
        vec3d center;
        for (const auto &p : points)
        {
            center[0] += p[0];
            center[1] += p[1];
            center[2] += p[2];
        }
        int n = points.size();
        center[0] /= n;
        center[1] /= n;
        center[2] /= n;
        return center;
    }

    Sphere trivial_sphere(const std::vector<vec3d> &points)
    {
        if (points.empty())
            return Sphere();
        if (points.size() == 1)
            return Sphere(points[0], 0);

        if (points.size() == 2)
        {
            vec3d center{(points[0][0] + points[1][0]) / 2,
                         (points[0][1] + points[1][1]) / 2,
                         (points[0][2] + points[1][2]) / 2};
            return Sphere(center, distance(center, points[0]));
        }

        // For 3 and 4 points, use barycenter as an approximation
        vec3d center = barycenter(points);
        double radius = 0;
        for (const auto &p : points)
        {
            radius = std::max(radius, distance(center, p));
        }
        return Sphere(center, radius);
    }

    bool is_inside_sphere(const vec3d &p, const Sphere &sphere)
    {
        return distance(p, sphere.center) <= sphere.radius + 1e-10;
    }

    Sphere welzl_helper(std::vector<vec3d> &P, std::vector<vec3d> R, int n)
    {
        if (n == 0 || R.size() == 4)
        {
            return trivial_sphere(R);
        }

        // Choose a random point
        int idx = std::rand() % n;
        vec3d p = P[idx];
        std::swap(P[idx], P[n - 1]);

        // Recursively compute the smallest sphere without p
        Sphere sphere = welzl_helper(P, R, n - 1);

        // If p is not inside the sphere, it must be on the boundary
        if (!is_inside_sphere(p, sphere))
        {
            R.push_back(p);
            sphere = welzl_helper(P, R, n - 1);
        }

        return sphere;
    }

    // Sphere welzl(std::vector<vec3d> &P)
    // {
    //     unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //     std::shuffle(P.begin(), P.end(), std::default_random_engine(seed));
    //     return welzl_helper(P, {}, P.size());
    // }
    std::vector<vec3d> find_extreme_points(const std::vector<vec3d> &P)
    {
        vec3d min_x = *std::min_element(P.begin(), P.end(), [](const vec3d &a, const vec3d &b)
                                        { return a[0] < b[0]; });
        vec3d max_x = *std::max_element(P.begin(), P.end(), [](const vec3d &a, const vec3d &b)
                                        { return a[0] < b[0]; });
        vec3d min_y = *std::min_element(P.begin(), P.end(), [](const vec3d &a, const vec3d &b)
                                        { return a[1] < b[1]; });
        vec3d max_y = *std::max_element(P.begin(), P.end(), [](const vec3d &a, const vec3d &b)
                                        { return a[1] < b[1]; });
        vec3d min_z = *std::min_element(P.begin(), P.end(), [](const vec3d &a, const vec3d &b)
                                        { return a[2] < b[2]; });
        vec3d max_z = *std::max_element(P.begin(), P.end(), [](const vec3d &a, const vec3d &b)
                                        { return a[2] < b[2]; });

        return {min_x, max_x, min_y, max_y, min_z, max_z};
    }

    // Sphere welzl(std::vector<vec3d> &P, int num_runs)
    // {
    //     std::vector<vec3d> extreme_points = find_extreme_points(P);
    //     Sphere best_sphere = trivial_sphere(extreme_points);

    //     for (int i = 0; i < num_runs; ++i)
    //     {
    //         Sphere current_sphere = welzl_helper(P, extreme_points, P.size());
    //         if (current_sphere.radius < best_sphere.radius)
    //         {
    //             best_sphere = current_sphere;
    //         }
    //     }

    //     // Final refinement
    //     for (const vec3d &p : P)
    //     {
    //         if (!is_inside_sphere(p, best_sphere))
    //         {
    //             std::vector<vec3d> R = {p};
    //             best_sphere = welzl_helper(P, R, P.size());
    //         }
    //     }
    //     // best_sphere.radius *= 0.8;

    //     // Decide the radius based on the same-volume criterion
    //     double volume = 4.0 / 3.0 * M_PI * best_sphere.radius * best_sphere.radius * best_sphere.radius;


    //     return best_sphere;
    // }

    Sphere welzl(Model &mesh, int num_runs)
    {
        std::vector<vec3d> P = mesh.points;
        std::vector<vec3d> extreme_points = find_extreme_points(P);
        Sphere best_sphere = trivial_sphere(extreme_points);

        for (int i = 0; i < num_runs; ++i)
        {
            Sphere current_sphere = welzl_helper(P, extreme_points, P.size());
            if (current_sphere.radius < best_sphere.radius)
            {
                best_sphere = current_sphere;
            }
        }

        // Final refinement
        for (const vec3d &p : P)
        {
            if (!is_inside_sphere(p, best_sphere))
            {
                std::vector<vec3d> R = {p};
                best_sphere = welzl_helper(P, R, P.size());
            }
        }

        // Decide the radius based on the same-volume criterion
        double v_mesh = MeshVolume(mesh);
        best_sphere.radius = 1.2 * std::pow(3 * v_mesh / (4 * M_PI), 1.0 / 3.0);

        return best_sphere;
    }
}