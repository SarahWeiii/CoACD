#include "shape.h"


namespace coacd 
{

Plane::Plane()
{
    pFlag = false;
}

bool SamePointDetect(vec3d p0, vec3d p1)
{
    double dx, dy, dz;
    dx = fabs(p0[0] - p1[0]);
    dy = fabs(p0[1] - p1[1]);
    dz = fabs(p0[2] - p1[2]);
    if (dx < 1e-5 && dy < 1e-5 && dz < 1e-5)
        return true;
    return false;
}

vec3d CalFaceNormal(vec3d p1, vec3d p2, vec3d p3)
{
    vec3d v, w, n, normal;
    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];
    w[0] = p3[0] - p1[0];
    w[1] = p3[1] - p1[1];
    w[2] = p3[2] - p1[2];

    n = CrossProduct(v, w);

    normal[0] = n[0] / sqrt(pow(n[0], 2) + pow(n[1], 2) + pow(n[2], 2));
    normal[1] = n[1] / sqrt(pow(n[0], 2) + pow(n[1], 2) + pow(n[2], 2));
    normal[2] = n[2] / sqrt(pow(n[0], 2) + pow(n[1], 2) + pow(n[2], 2));

    return normal;
}

double Area(vec3d p0, vec3d p1, vec3d p2)
{
    return 0.5 * sqrt(pow(p1[0] * p0[1] - p2[0] * p0[1] - p0[0] * p1[1] + p2[0] * p1[1] + p0[0] * p2[1] - p1[0] * p2[1], 2) +
                      pow(p1[0] * p0[2] - p2[0] * p0[2] - p0[0] * p1[2] + p2[0] * p1[2] + p0[0] * p2[2] - p1[0] * p2[2], 2) +
                      pow(p1[1] * p0[2] - p2[1] * p0[2] - p0[1] * p1[2] + p2[1] * p1[2] + p0[1] * p2[2] - p1[1] * p2[2], 2));
}

double Volume(vec3d p1, vec3d p2, vec3d p3)
{
    double v321 = p3[0] * p2[1] * p1[2];
    double v231 = p2[0] * p3[1] * p1[2];
    double v312 = p3[0] * p1[1] * p2[2];
    double v132 = p1[0] * p3[1] * p2[2];
    double v213 = p2[0] * p1[1] * p3[2];
    double v123 = p1[0] * p2[1] * p3[2];
    return (1.0 / 6.0) * (-v321 + v231 + v312 - v132 - v213 + v123);
}

void Diagonalize(const double (&A)[3][3], double Q[3][3], double D[3][3])
{
    // A must be a symmetric matrix.
    // returns Q and D such that
    // Diagonal matrix D = QT * A * Q;  and  A = Q*D*QT
    const int32_t maxsteps = 24; // certainly wont need that many.
    int32_t k0, k1, k2;
    double o[3], m[3];
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    double jr[4];
    double sqw, sqx, sqy, sqz;
    double tmp1, tmp2, mq;
    double AQ[3][3];
    double thet, sgn, t, c;
    for (int32_t i = 0; i < maxsteps; ++i)
    {
        // quat to matrix
        sqx = q[0] * q[0];
        sqy = q[1] * q[1];
        sqz = q[2] * q[2];
        sqw = q[3] * q[3];
        Q[0][0] = (sqx - sqy - sqz + sqw);
        Q[1][1] = (-sqx + sqy - sqz + sqw);
        Q[2][2] = (-sqx - sqy + sqz + sqw);
        tmp1 = q[0] * q[1];
        tmp2 = q[2] * q[3];
        Q[1][0] = 2.0 * (tmp1 + tmp2);
        Q[0][1] = 2.0 * (tmp1 - tmp2);
        tmp1 = q[0] * q[2];
        tmp2 = q[1] * q[3];
        Q[2][0] = 2.0 * (tmp1 - tmp2);
        Q[0][2] = 2.0 * (tmp1 + tmp2);
        tmp1 = q[1] * q[2];
        tmp2 = q[0] * q[3];
        Q[2][1] = 2.0 * (tmp1 + tmp2);
        Q[1][2] = 2.0 * (tmp1 - tmp2);

        // AQ = A * Q
        AQ[0][0] = Q[0][0] * A[0][0] + Q[1][0] * A[0][1] + Q[2][0] * A[0][2];
        AQ[0][1] = Q[0][1] * A[0][0] + Q[1][1] * A[0][1] + Q[2][1] * A[0][2];
        AQ[0][2] = Q[0][2] * A[0][0] + Q[1][2] * A[0][1] + Q[2][2] * A[0][2];
        AQ[1][0] = Q[0][0] * A[0][1] + Q[1][0] * A[1][1] + Q[2][0] * A[1][2];
        AQ[1][1] = Q[0][1] * A[0][1] + Q[1][1] * A[1][1] + Q[2][1] * A[1][2];
        AQ[1][2] = Q[0][2] * A[0][1] + Q[1][2] * A[1][1] + Q[2][2] * A[1][2];
        AQ[2][0] = Q[0][0] * A[0][2] + Q[1][0] * A[1][2] + Q[2][0] * A[2][2];
        AQ[2][1] = Q[0][1] * A[0][2] + Q[1][1] * A[1][2] + Q[2][1] * A[2][2];
        AQ[2][2] = Q[0][2] * A[0][2] + Q[1][2] * A[1][2] + Q[2][2] * A[2][2];
        // D = Qt * AQ
        D[0][0] = AQ[0][0] * Q[0][0] + AQ[1][0] * Q[1][0] + AQ[2][0] * Q[2][0];
        D[0][1] = AQ[0][0] * Q[0][1] + AQ[1][0] * Q[1][1] + AQ[2][0] * Q[2][1];
        D[0][2] = AQ[0][0] * Q[0][2] + AQ[1][0] * Q[1][2] + AQ[2][0] * Q[2][2];
        D[1][0] = AQ[0][1] * Q[0][0] + AQ[1][1] * Q[1][0] + AQ[2][1] * Q[2][0];
        D[1][1] = AQ[0][1] * Q[0][1] + AQ[1][1] * Q[1][1] + AQ[2][1] * Q[2][1];
        D[1][2] = AQ[0][1] * Q[0][2] + AQ[1][1] * Q[1][2] + AQ[2][1] * Q[2][2];
        D[2][0] = AQ[0][2] * Q[0][0] + AQ[1][2] * Q[1][0] + AQ[2][2] * Q[2][0];
        D[2][1] = AQ[0][2] * Q[0][1] + AQ[1][2] * Q[1][1] + AQ[2][2] * Q[2][1];
        D[2][2] = AQ[0][2] * Q[0][2] + AQ[1][2] * Q[1][2] + AQ[2][2] * Q[2][2];
        o[0] = D[1][2];
        o[1] = D[0][2];
        o[2] = D[0][1];
        m[0] = fabs(o[0]);
        m[1] = fabs(o[1]);
        m[2] = fabs(o[2]);

        k0 = (m[0] > m[1] && m[0] > m[2]) ? 0 : (m[1] > m[2]) ? 1
                                                              : 2; // index of largest element of offdiag
        k1 = (k0 + 1) % 3;
        k2 = (k0 + 2) % 3;
        if (o[k0] == 0.0)
        {
            break; // diagonal already
        }
        thet = (D[k2][k2] - D[k1][k1]) / (2.0 * o[k0]);
        sgn = (thet > 0.0) ? 1.0 : -1.0;
        thet *= sgn;                                                         // make it positive
        t = sgn / (thet + ((thet < 1.E6) ? sqrt(thet * thet + 1.0) : thet)); // sign(T)/(|T|+sqrt(T^2+1))
        c = 1.0 / sqrt(t * t + 1.0);                                         //  c= 1/(t^2+1) , t=s/c
        if (c == 1.0)
        {
            break; // no room for improvement - reached machine precision.
        }
        jr[0] = jr[1] = jr[2] = jr[3] = 0.0;
        jr[k0] = sgn * sqrt((1.0 - c) / 2.0); // using 1/2 angle identity sin(a/2) = sqrt((1-cos(a))/2)
        jr[k0] *= -1.0;                       // since our quat-to-matrix convention was for v*M instead of M*v
        jr[3] = sqrt(1.0 - jr[k0] * jr[k0]);
        if (jr[3] == 1.0)
        {
            break; // reached limits of floating point precision
        }
        q[0] = (q[3] * jr[0] + q[0] * jr[3] + q[1] * jr[2] - q[2] * jr[1]);
        q[1] = (q[3] * jr[1] - q[0] * jr[2] + q[1] * jr[3] + q[2] * jr[0]);
        q[2] = (q[3] * jr[2] + q[0] * jr[1] - q[1] * jr[0] + q[2] * jr[3]);
        q[3] = (q[3] * jr[3] - q[0] * jr[0] - q[1] * jr[1] - q[2] * jr[2]);
        mq = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] /= mq;
        q[1] /= mq;
        q[2] /= mq;
        q[3] /= mq;
    }
}
}