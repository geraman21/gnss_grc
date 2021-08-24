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
    AcqResults(int _PRN, float _carrFreq, float _codePhase, float _peakMetric);
    AcqResults();
    float carrFreq{};
    float codePhase{};
    float peakMetric{};
    int PRN{};
    int channelNumber{};
};

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */