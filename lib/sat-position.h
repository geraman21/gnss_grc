/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SATELITE_POSITION
#define SATELITE_POSITION

#include "ephemeris.h"
#include <gnss/tracking_ff.h>
#include <vector>

class SatPosition {

public:
  bool isActive = false;
  double pos1;
  double pos2;
  double pos3;
  double satClkCorr;

  SatPosition();
  SatPosition(double transmitTime, Ephemeris eph);
  SatPosition(double _pos1, double _pos2, double _pos3);
};

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */
