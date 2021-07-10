/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_NAV_DECODING_IMPL_H
#define INCLUDED_GNSS_NAV_DECODING_IMPL_H
#include <gnss/nav_decoding.h>
#include <deque>
#include <string>
#include <vector>

namespace gr {
namespace gnss {

class nav_decoding_impl : public nav_decoding
{
private:
    int channel;
    int unsigned iterator{ 0 };
    int test = 0;
    int samplesForPreamble;
    int subframeStart = 0;
    int parityResult;
    int buffer[37000];
    double codePhaseMs;
    std::deque<int> travelTimeQue;
    int bitCounter{ 0 }, bitSum{ 0 };
    double result = 0;

public:
    nav_decoding_impl(int channelNum, int codePhase);
    ~nav_decoding_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_DECODING_IMPL_H */
