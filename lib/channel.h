/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SATELITE_CHANNEL
#define SATELITE_CHANNEL

#include <gnss/tracking_ff.h>
#include <vector>

class Channel  {
        
    public: 
        int prn;
        float acquiredFreq;
        int codePhase;
        char status;
        Channel();
        Channel(int _prn, float _acquiredFreq, int _codePhase, char _status);
};

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */

