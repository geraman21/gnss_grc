#include "helper-functions.h"
#include <tuple>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

double check_t(double time);
std::tuple<Vector4d, vector<double>, vector<double>, vector<double>> leastSquarePos(vector<SatPosition> satpos, vector<float> obs, long int c);
tuple<double, double, double> cart2geo(double X, double Y, double Z, int i);