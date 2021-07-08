/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_DECODING_TEST_IMPL_H
#define INCLUDED_GNSS_DECODING_TEST_IMPL_H

#include <gnss/decoding_test.h>
#include <deque>
#include <string>
#include <vector>
namespace gr {
namespace gnss {

class decoding_test_impl : public decoding_test
{
private:
    std::string channel;
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
    // void printMessage(std::string msg);


public:
    decoding_test_impl(int channelNum, int codePhase);
    ~decoding_test_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_DECODING_TEST_IMPL_H */
