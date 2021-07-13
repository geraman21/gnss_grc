/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SATELITE_POSITION
#define SATELITE_POSITION

#include <gnss/tracking_ff.h>
#include <vector>
#include "ephemeris.h"

class SatPosition
{

public:
    bool isActive = false;
    double pos1;
    double pos2;
    double pos3;
    double satClkCorr;

    SatPosition();
    SatPosition(double transmitTime, Ephemeris eph);
};

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */
