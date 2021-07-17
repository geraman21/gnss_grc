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

SatPosition e_r_corr(double traveltime, SatPosition X_sat)
{
    double Omegae_dot = 7.292115147e-5; // rad/sec

    // Find rotation angle
    double omegatau = Omegae_dot * traveltime;
    Matrix<double, 3, 3> R3;
    R3 << cos(omegatau), sin(omegatau), 0,
        -sin(omegatau), cos(omegatau), 0,
        0, 0, 1;
    MatrixXd pos(3, 0);
    pos << X_sat.pos1, X_sat.pos2, X_sat.pos3;
    MatrixXd result;
    result = R3 * pos;

    X_sat.pos1 = result(0, 0);
    X_sat.pos2 = result(1, 0);
    X_sat.pos3 = result(2, 0);

    return X_sat;
}

tuple<vector<double>, vector<double>, vector<double>, vector<double>> leastSquarePos(vector<SatPosition> satpos, vector<float> obs, long int c)
{

    float dtr = M_PI / 180;
    int trop;
    SatPosition Rot_X;
    int nmbOfIterations = 7;
    int nmbOfSatellites = satpos.size();

    vector<double> pos(4, 0);
    vector<double> el(nmbOfSatellites, 0);
    vector<double> az(nmbOfSatellites, 0);
    vector<double> dop(5, 0);

    vector<vector<double>> A(nmbOfSatellites, vector<double>(4));
    vector<double> omc(nmbOfSatellites);

    for (int iter = 0; iter < nmbOfIterations; iter++)
    {
        for (int i = 0; i < nmbOfSatellites; i++)
        {
            if (iter == 0)
            {
                trop = 2;
                Rot_X = satpos.at(i);
            }
            else
            {
                double rho2 = pow((satpos.at(i).pos1 - pos.at(0)), 2) + pow(satpos.at(i).pos2 - pos.at(1), 2) + pow(satpos.at(i).pos3 - pos.at(2), 2);
                double travelTime = sqrt(rho2) / c;
            }
        }
    }
}