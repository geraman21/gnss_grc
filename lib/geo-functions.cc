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

MatrixXd e_r_corr(double traveltime, MatrixXd X_sat)
{
    double Omegae_dot = 7.292115147e-5; // rad/sec

    // Find rotation angle
    double omegatau = Omegae_dot * traveltime;
    Matrix<double, 3, 3> R3;
    R3 << cos(omegatau), sin(omegatau), 0,
        -sin(omegatau), cos(omegatau), 0,
        0, 0, 1;
    Vector3d pos;
    pos << X_sat(0, 0), X_sat(1, 0), X_sat(2, 0);
    MatrixXd result;
    result = R3 * pos;

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
}

tuple<vector<double>, vector<double>, vector<double>, vector<double>> leastSquarePos(vector<SatPosition> satpos, vector<float> obs, long int c)
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

    vector<vector<double>> A(nmbOfSatellites, vector<double>(4));
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

                auto [azi, eli, dist] = topocent(pos, Rot_X - pos);
                az.at(i) = azi;
                el.at(i) = eli;
                omc(i) = obs.at(i) - (Rot_X - pos.head(3)).norm() - pos(4);

                A.row(i) << (-(Rot_X(0) - pos(0))) / obs.at(i),
                    (-(Rot_X(1) - pos(1))) / obs.at(i),
                    (-(Rot_X(2) - pos(2))) / obs.at(i),
                    1;
            }
        }
        // Solve Ax = b. Result stored in x. Matlab: x = A \ b.
        // x = A.lu()  .solve(b));
        MatrixXd x;
        x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(omc);
        pos = pos + x;
    }
    //     Q       = inv(A'*A);
    MatrixXd Q = (A.transpose() * A).inverse();
    dop.at(0) = sqrt(Q.trace());
    dop.at(1) = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    dop.at(2) = sqrt(Q(0, 0) + Q(1, 1));
    dop.at(3) = sqrt(Q(2, 2));
    dop.at(4) = sqrt(Q(3, 3));
}