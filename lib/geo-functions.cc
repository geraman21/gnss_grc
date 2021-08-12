#include <vector>
#include "sat-position.h"
#include <math.h>
#include <tuple>
#include <Eigen/Dense>
#include "helper-functions.h"

using namespace Eigen;
using namespace std;

double check_t(double time)
{
    double corrTime = time;
    int half_weeK = 302400;
    if (time > half_weeK)
        corrTime = time - (2 * half_weeK);
    else if (time < -half_weeK)
        corrTime = time + (2 * half_weeK);

    return corrTime;
}

MatrixXd e_r_corr(double traveltime, Vector3d X_sat)
{
    double Omegae_dot = 7.292115147e-5; // rad/sec

    // Find rotation angle
    double omegatau = Omegae_dot * traveltime;
    Matrix<double, 3, 3> R3;
    R3 << cos(omegatau), sin(omegatau), 0,
        -sin(omegatau), cos(omegatau), 0,
        0, 0, 1;

    MatrixXd result;
    result = R3 * X_sat;

    return result;
}

tuple<double, double, double> togeod(double a, double finv, double X, double Y, double Z)
{
    // TOGEOD   Subroutine to calculate geodetic coordinates latitude, longitude,
    //          height given Cartesian coordinates X,Y,Z, and reference ellipsoid
    //          values semi-major axis (a) and the inverse of flattening (finv).
    //
    // [dphi, dlambda, h] = togeod(a, finv, X, Y, Z);
    //
    //   The units of linear parameters X,Y,Z,a must all agree (m,km,mi,ft,..etc)
    //   The output units of angular quantities will be in decimal degrees
    //   (15.5 degrees not 15 deg 30 min). The output units of h will be the
    //   same as the units of X,Y,Z,a.
    //
    //    Inputs:
    //        a           - semi-major axis of the reference ellipsoid
    //        finv        - inverse of flattening of the reference ellipsoid
    //        X,Y,Z       - Cartesian coordinates
    //
    //    Outputs:
    //        dphi        - latitude
    //        dlambda     - longitude
    //        h           - height above reference ellipsoid

    double h = 0;
    double tolsq = 1.e-10;
    double maxit = 10;

    //  compute radians-to-degree factor
    double rtd = 180 / M_PI;

    double esq;
    finv < 1.e-20 ? esq = 0 : esq = (2 - 1 / finv) / finv;
    double oneesq = 1 - esq;

    //      first guess
    //  P is distance from spin axis
    double P = sqrt(pow(X, 2) + pow(Y, 2));
    //  direct calculation of longitude

    double dlambda;

    P > 1.e-20 ? dlambda = atan2(Y, X) *rtd : dlambda = 0;

    dlambda < 0 ? dlambda += 360 : true;

    //  r is distance from origin (0,0,0)
    double r = sqrt(pow(P, 2) + pow(Z, 2));

    double sinphi;
    r > 1.e-20 ? sinphi = Z / r : sinphi = 0;

    double dphi = asin(sinphi);

    if (r < 1.e-20)
    {
        return {dphi, dlambda, h};
    }

    h = r - a * (1 - sinphi * sinphi / finv);

    for (int i = 1; i <= maxit; i++)
    {
        double sinphi = sin(dphi);
        double cosphi = cos(dphi);

        //    compute radius of curvature in prime vertical direction
        double N_phi = a / sqrt(1 - esq * sinphi * sinphi);

        // compute residuals in P and Z
        double dP = P - (N_phi + h) * cosphi;
        double dZ = Z - (N_phi * oneesq + h) * sinphi;

        //    update height and latitude
        h = h + (sinphi * dZ + cosphi * dP);
        dphi = dphi + (cosphi * dZ - sinphi * dP) / (N_phi + h);

        //  test for convergence
        if (dP * dP + dZ * dZ < tolsq)
            break;

        if (i == maxit)
        {
            cout << "Problem in TOGEOD, did not converge" << endl;
        }
    }
    dphi = dphi * rtd;

    return {dphi, dlambda, h};
}

tuple<double, double, double> topocent(Vector3d X, Vector3d dx)
{
    //     TOPOCENT  Transformation of vector dx into topocentric coordinate
    //           system with origin at X.
    //           Both parameters are 3 by 1 vectors.
    //
    // [Az, El, D] = topocent(X, dx);
    //
    //    Inputs:
    //        X           - vector origin corrdinates (in ECEF system [X; Y; Z;])
    //        dx          - vector ([dX; dY; dZ;]).
    //
    //    Outputs:
    //        D           - vector length. Units like units of the input
    //        Az          - azimuth from north positive clockwise, degrees
    //        El          - elevation angle, degrees

    float dtr = M_PI / 180;
    double Az, El, D;
    auto [phi, lambda, h] = togeod(6378137, 298.257223563, X(0), X(1), X(2));

    double cl = cos(lambda * dtr);
    double sl = sin(lambda * dtr);
    double cb = cos(phi * dtr);
    double sb = sin(phi * dtr);

    Matrix3d F;
    F << -sl, -sb * cl, cb * cl,
        cl, -sb * sl, cb * sl,
        0, cb, sb;

    MatrixXd local_vector;
    local_vector = F.adjoint() * dx;
    double E = local_vector(0);
    double N = local_vector(1);
    double U = local_vector(2);

    double hor_dis = sqrt(pow(E, 2) + pow(N, 2));

    if (hor_dis < 1.e-20)
    {
        Az = 0;
        El = 90;
    }
    else
    {
        Az = atan2(E, N) / dtr;
        El = atan2(U, hor_dis) / dtr;
    }
    Az < 0 ? Az += 360 : true;

    D = sqrt(pow(dx(0), 2) + pow(dx(1), 2) + pow(dx(2), 2));

    return {Az, El, D};
}

tuple<Vector4d, vector<double>, vector<double>, vector<double>> leastSquarePos(vector<SatPosition> satpos, vector<float> obs, long int c)

{

    float dtr = M_PI / 180;
    int trop;
    Vector3d Rot_X;

    int nmbOfIterations = 7;
    const int nmbOfSatellites = satpos.size();

    Vector4d pos;
    pos.setZero();
    MatrixXd A(nmbOfSatellites, 4);
    vector<double> el(nmbOfSatellites, 0);
    vector<double> az(nmbOfSatellites, 0);
    vector<double> dop(5, 0);

    VectorXd omc(nmbOfSatellites);

    for (int iter = 0; iter < nmbOfIterations; iter++)
    {
        for (int i = 0; i < nmbOfSatellites; i++)
        {
            Vector3d X;
            X << satpos.at(i).pos1, satpos.at(i).pos2, satpos.at(i).pos3;

            if (iter == 0)
            {
                trop = 2;
                Rot_X << satpos.at(i).pos1, satpos.at(i).pos2, satpos.at(i).pos3;
            }
            else
            {
                double rho2 = pow((X(0) - pos(0)), 2) + pow(X(1) - pos(1), 2) + pow(X(2) - pos(2), 2);
                double travelTime = sqrt(rho2) / c;
                // Correct satellite position (do to earth rotation)
                Rot_X = e_r_corr(travelTime, X);
                auto [azi, eli, dist] = topocent(pos.head(3), Rot_X - pos.head(3));
                az.at(i) = azi;
                el.at(i) = eli;
            }
            omc(i) = obs.at(i) - (Rot_X - pos.head(3)).norm() - pos(4);

            A.row(i) << (-(Rot_X(0) - pos(0))) / obs.at(i),
                (-(Rot_X(1) - pos(1))) / obs.at(i),
                (-(Rot_X(2) - pos(2))) / obs.at(i),
                1;
        }

        MatrixXd x;
        x = A.colPivHouseholderQr().solve(omc);
        // if (iter == 0)
        // {
        //     cout << "A ++++++++++++" << endl;
        //     cout << A << endl;
        //     cout << "OMC ++++++++++" << endl;
        //     cout << omc << endl;
        //     cout << "x ++++++++++++" << endl;
        //     cout << x << endl;
        // }
        pos = pos + x;
    }
    //     Q       = inv(A'*A);
    MatrixXd Q = (A.adjoint() * A).inverse();
    dop.at(0) = sqrt(Q.trace());
    dop.at(1) = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    dop.at(2) = sqrt(Q(0, 0) + Q(1, 1));
    dop.at(3) = sqrt(Q(2, 2));
    dop.at(4) = sqrt(Q(3, 3));

    return {pos, el, az, dop};
}

tuple<double, double, double> cart2geo(double X, double Y, double Z, int index)
{
    // CART2GEO Conversion of Cartesian coordinates (X,Y,Z) to geographical
    // coordinates (phi, lambda, h) on a selected reference ellipsoid.
    //
    // [phi, lambda, h] = cart2geo(X, Y, Z, i);
    //
    //    Choices i of Reference Ellipsoid for Geographical Coordinates
    //           	  1. International Ellipsoid 1924
    // 	          2. International Ellipsoid 1967
    // 	          3. World Geodetic System 1972
    // 	          4. Geodetic Reference System 1980
    // 	          5. World Geodetic System 1984

    // Kai Borre 10-13-98
    // Copyright (c) by Kai Borre
    // Revision: 1.0   Date: 1998/10/23
    //
    //  CVS record:
    //  $Id: cart2geo.m,v 1.1.2.3 2007/01/29 15:22:49 dpl Exp $
    // ==========================================================================
    int i = index - 1;
    double a[]{6378388, 6378160, 6378135, 6378137, 6378137};
    double f[]{1 / 297, 1 / 298.247, 1 / 298.26, 1 / 298.257222101, 1 / 298.257223563};

    double lambda = atan2(Y, X);
    double ex2 = (2 - f[i]) * f[i] / (pow((1 - f[i]), 2));
    double c = a[i] * sqrt(1 + ex2);
    double phi = atan(Z / (sqrt(X * X + Y * Y) * (1 - (2 - f[i])) * f[i]));

    double h = 0.1;
    double oldh = 0;
    double N;
    int iterations = 0;
    while (abs(h - oldh) > 1.e-12)
    {
        oldh = h;
        N = c / sqrt(1 + ex2 * pow(cos(phi), 2));
        phi = atan(Z / (sqrt(X * X + Y * Y) * (1 - (2 - f[i]) * f[i] * N / (N + h))));
        h = sqrt(X * X + Y * Y) / cos(phi) - N;

        iterations = iterations + 1;
        if (iterations > 100)
        {
            cout << "Failed to approximate h with desired precision";
            break;
        }
    }

    phi = phi * 180 / M_PI;
    lambda = lambda * 180 / M_PI;
    return {phi, lambda, h};
}

int findUtmZone(double latitude, double longitude)
{
    // Check value bounds
    if ((longitude > 180) || (longitude < -180))
    {
        cout << "Longitude value exceeds limits (-180:180)" << endl;
        return -1;
    }
    if (latitude > 84 || latitude < -80)
    {
        cout << "Latitude value exceeds limits (-80:84)." << endl;
        return -1;
    }

    //  Find zone ==============================================================

    //  Start at 180 deg west = -180 deg

    int utmZone = trunc((180 + longitude) / 6) + 1;

    //  Correct zone numbers for particular areas
    if (latitude > 72)
    {
        if ((longitude >= 0) && (longitude < 9))
            utmZone = 31;
        else if ((longitude >= 9) && (longitude < 21))
            utmZone = 33;
        else if ((longitude >= 21) && (longitude < 33))
            utmZone = 35;
        else if ((longitude >= 33) && (longitude < 42))
            utmZone = 37;
    }
    else if ((latitude >= 56) && (latitude < 64))
    {
        if ((longitude >= 3) && (longitude < 12))
            utmZone = 32;
    }

    return utmZone;
}

// double clsin(double ar, int degree, double argument)
// {
//     double cos_arg = 2 * cos(argument);
//     double hr1 = 0;
//     double hr = 0;

//     for (int t = degree; t >= -1; t--)
//     {
//         double hr2 = hr1;
//         hr1 = hr;
//         hr = ar(t) + cos_arg * hr1 - hr2;
//     }

//     result = hr * sin(argument);
// }

// tuple<double, double, double> cart2utm(double X, double Y, double Z, int zone)
// {
//     double a = 6378388;
//     double f = 1 / 297;
//     double ex2 = (2 - f) * f / pow((1 - f), 2);
//     double c = a * sqrt(1 + ex2);
//     Vector3d vec;
//     vec << X, Y, Z - 4.5;
//     double alpha = 0.756e-6;
//     Matrix3d R;
//     R << 1, -alpha, 0,
//         alpha, 1, 0,
//         0, 0, 1;
//     Vector3d trans;
//     trans << 89.5, 93.8, 127.6;
//     double scale = 0.9999988;
//     MatrixXd v;
//     v = scale * R * vec + trans;
//     //  coordinate vector in ED50
//     double L = atan2(v(1), v(0));
//     double N1 = 6395000;
//     //  preliminary value
//     double B = atan2(v(2) / (pow((1 - f), 2) * N1), v.head(2).norm() / N1);
//     //  preliminary value
//     double U = 0.1;
//     double oldU = 0;

//     int iterations = 0;
//     while (abs(U - oldU) > 1.e-4)
//     {
//         oldU = U;
//         N1 = c / sqrt(1 + ex2 * (pow(cos(B), 2)));
//         B = atan2(v(2) / (pow((1 - f), 2) * N1 + U), v.head(2).norm() / (N1 + U));
//         U = v.head(2).norm() / cos(B) - N1;

//         iterations = iterations + 1;
//         if (iterations > 100)
//         {
//             cout << "Failed to approximate U with desired precision." << endl;
//             break;
//         }
//     }

//     //  Normalized meridian quadrant, KW p.50(96), p.19(38b), p.5(21)
//     double m0 = 0.0004;
//     double n = f / (2 - f);
//     double m = n * n * (1 / 4 + n * n / 64);
//     double w = (a * (-n - m0 + m * (1 - m0))) / (1 + n);
//     double Q_n = a + w;

//     //  Easting and longitude of central meridian double E0 = 500000;
//     double L0 = (zone - 30) * 6 - 3;

//     // Check tolerance for reverse transformation
//     double tolutm = M_PI / 2 * 1.2e-10 * Q_n;
//     double tolgeo = 0.000040;

//     // With f = 1/297 we get

//     Vector4d bg;
//     bg << -3.37077907e-3,
//         4.73444769e-6,
//         -8.29914570e-9,
//         1.58785330e-11;

//     Vector4d gb;
//     gb << 3.37077588e-3,
//         6.62769080e-6,
//         1.78718601e-8,
//         5.49266312e-11;

//     Vector4d gtu;
//     gtu << 8.41275991e-4,
//         7.67306686e-7,
//         1.21291230e-9,
//         2.48508228e-12;

//     Vector4d utg;
//     utg << -8.41276339e-4,
//         -5.95619298e-8,
//         -1.69485209e-10,
//         -2.20473896e-13;

//     // Ellipsoidal latitude, longitude to spherical latitude, longitude
//     bool neg_geo = false;

//     if (B < 0)
//         neg_geo = true;

//     double Bg_r = abs(B);
//     // [res_clensin] = clsin(bg, 4, 2 * Bg_r);
//     // Bg_r = Bg_r + res_clensin;
//     // L0 = L0 * pi / 180;
//     // Lg_r = L - L0;

//     // % Spherical latitude, longitude to complementary spherical latitude % i.e.spherical N, E cos_BN = cos(Bg_r);
//     // Np = atan2(sin(Bg_r), cos(Lg_r) * cos_BN);
//     // Ep = atanh(sin(Lg_r) * cos_BN);

//     // % Spherical normalized N, E to ellipsoidal N, E Np = 2 * Np;
//     // Ep = 2 * Ep;
//     // [ dN, dE ] = clksin(gtu, 4, Np, Ep);
//     // Np = Np / 2;
//     // Ep = Ep / 2;
//     // Np = Np + dN;
//     // Ep = Ep + dE;
//     // N = Q_n * Np;
//     // E = Q_n * Ep + E0;

//     // if neg_geo
//     //     == 'TRUE ' N = -N + 20000000;
//     // end;

//     return {0, 0, 0};
// }