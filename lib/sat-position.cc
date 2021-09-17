#include "sat-position.h"
#include "geo-functions.h"
#include <cmath>
// #include <math.h>

SatPosition::SatPosition(double transmitTime, Ephemeris eph) {
  double gpsPi = 3.1415926535898;
  double Omegae_dot = 7.2921151467e-5; // Earth rotation rate, [rad/s]
  double GM = 3.986005e14;             // Universal gravitational constant times the mass of the
                                       // Earth, [m^3/s^2]
  double F = -4.442807633e-10;         // Constant, [sec/(meter)^(1/2)]
  if (eph.channelNumber != -1) {
    // Find initial satellite clock correction

    // Find time difference
    double dt = check_t(transmitTime - eph.t_oc);
    std::cout.precision(12);
    // std::cout << std::fixed << "    " << dt << std::endl;
    // Calculate clock correction
    satClkCorr = (eph.a_f2 * dt + eph.a_f1) * dt + eph.a_f0 - eph.T_GD;

    double time = transmitTime - satClkCorr;

    // Find Satellites position

    // Restore semi-major axis
    double a = eph.sqrtA * eph.sqrtA;
    // Time Correction
    double tk = check_t(time - eph.t_oe);

    // Initial mean motion
    double n0 = sqrt(GM / pow(a, 3));
    // Mean motion
    double n = n0 + eph.deltan;

    // Mean Anomaly
    double M = eph.M_0 + n * tk;
    // Reduce mean anomaly to between 0 and 360 deg
    M = fmod(M + 2 * gpsPi, 2 * gpsPi);

    // Initial guess of eccentric anomaly
    double E = M;

    // Iteratively compute eccentric anomaly
    for (int i = 0; i < 10; i++) {
      double E_old = E;
      E = M + eph.e * sin(E);
      double dE = fmod(E - E_old, 2 * gpsPi);

      if (abs(dE) < 1.0e-12)
        break;
    }

    // Reduce eccentric anomaly to between 0 and 360 deg
    E = fmod(E + 2 * gpsPi, 2 * gpsPi);

    // Compute relativistic correction term
    double dtr = F * eph.e * eph.sqrtA * sin(E);
    // Calculate the true anomaly
    double nu = atan2(sqrt(1 - pow(eph.e, 2)) * sin(E), cos(E) - eph.e);

    // Compute angle phi
    double phi = nu + eph.omega;
    phi = fmod(phi, 2 * gpsPi);

    // Correct argument of latitude
    double u = phi + eph.C_uc * cos(2 * phi) + eph.C_us * sin(2 * phi);
    // Correct radius
    double r = a * (1 - eph.e * cos(E)) + eph.C_rc * cos(2 * phi) + eph.C_rs * sin(2 * phi);
    // Correct inclination
    double i = eph.i_0 + eph.iDot * tk + eph.C_ic * cos(2 * phi) + eph.C_is * sin(2 * phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    double Omega = eph.omega_0 + (eph.omegaDot - Omegae_dot) * tk - Omegae_dot * eph.t_oe;
    // Reduce to between 0 and 360 deg
    Omega = fmod(Omega + 2 * gpsPi, 2 * gpsPi);

    // --- Compute satellite coordinates ------------------------------------
    pos1 = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    pos2 = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);
    pos3 = sin(u) * r * sin(i);

    satClkCorr = (eph.a_f2 * dt + eph.a_f1) * dt + eph.a_f0 - eph.T_GD + dtr;
    isActive = true;
  }
}

SatPosition::SatPosition(double _pos1, double _pos2, double _pos3)
    : pos1{_pos1}, pos2{_pos2}, pos3{_pos3} {}

SatPosition::SatPosition() {}