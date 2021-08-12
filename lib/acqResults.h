/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef ACQ_RESULTS
#define ACQ_RESULTS

#include <vector>

class AcqResults
{

public:
    AcqResults();
    std::vector<double> carrFreq;
    std::vector<double> codePhase;
    std::vector<double> peakMetric;
};

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */