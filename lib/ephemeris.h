/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SATELITE_EPHEMERIS
#define SATELITE_EPHEMERIS

#include <vector>

class Ephemeris
{

public:
    void printEphemeris();
    Ephemeris(std::vector<int>& navBits, int channel);
    Ephemeris();
    int channelNumber = -1;
    double gpsPi = 3.1415926535898;
    int weekNumber;
    int accuracy;
    int health;
    double T_GD;
    float IODC;
    int t_oc;
    double a_f2;
    double a_f1;
    double a_f0;

    int IODE_sf2;
    double C_rs;
    double deltan;
    double M_0;
    double C_uc;
    double e;
    double C_us;
    double sqrtA;
    int t_oe;

    double C_ic;
    double omega_0;
    double C_is;
    double i_0;
    double C_rc;
    double omega;
    double omegaDot;
    int IODE_sf3;
    double iDot;

    int TOW;
};

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */