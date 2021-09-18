#include "helper-functions.h"
#include <Eigen/Dense>
#include <tuple>

using namespace Eigen;
using namespace std;

double check_t(double time);
double tropo(double sinel, double hsta, double p, double tkel, double hum, double hp, double htkel,
             double hhum);
std::tuple<Vector4d, vector<double>, vector<double>, vector<double>>
leastSquarePos(vector<SatPosition> satpos, vector<double> obs, long int c);
tuple<double, double, double> cart2geo(double X, double Y, double Z, int i);