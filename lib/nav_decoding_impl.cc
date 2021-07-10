/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ephemeris.h"
#include "helper-functions.h"
#include "nav_decoding_impl.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <algorithm>
#include <string>
#include <vector>

namespace gr {
namespace gnss {


using input_type = float;
using output_type = float;
nav_decoding::sptr nav_decoding::make(int channelNum, int codePhase)
{
    return gnuradio::make_block_sptr<nav_decoding_impl>(channelNum, codePhase);
}


/*
 * The private constructor
 */
nav_decoding_impl::nav_decoding_impl(int channelNum, int codePhase)
    : gr::sync_decimator("nav_decoding",
                         gr::io_signature::make(
                             1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                         gr::io_signature::make(1 /* min outputs */,
                                                1 /*max outputs */,
                                                sizeof(output_type)),
                         500 /*<+decimation+>*/)
{
    channel = channelNum;
    message_port_register_out(pmt::string_to_symbol("ephemeris"));

    samplesForPreamble = 14000;
    codePhaseMs = (codePhase * 1.0) / 38192;
    int reversePreambleShort[]{ 1, 1, 0, 1, 0, 0, 0, 1 };
    int reversePreamble[160];
    for (int i = 0; i < 8; i++) {
        for (int n = i * 20; n < ((i + 1) * 20); n++) {
            if (reversePreambleShort[i] == 1)
                reversePreamble[n] = 1;
            else
                reversePreamble[n] = -1;
        }
    }
}

/*
 * Our virtual destructor.
 */
nav_decoding_impl::~nav_decoding_impl() {}

int nav_decoding_impl::work(int noutput_items,
                            gr_vector_const_void_star& input_items,
                            gr_vector_void_star& output_items)
{
    const input_type* in = reinterpret_cast<const input_type*>(input_items[0]);
    output_type* out = reinterpret_cast<output_type*>(output_items[0]);

    for (int j = 0; j < noutput_items; j++) {
        for (int i = j * 500; i < j * 500 + 500; i++) {

            // Add data to travel time queue

            if (iterator > samplesForPreamble - 1) {
                travelTimeQue.pop_front();
                in[i] > 0 ? travelTimeQue.push_back(1) : travelTimeQue.push_back(-1);
            }

            // Collect enough data into a buffer to calculate Ephemerides later
            if (iterator < 37000) {
                if (in[i] > 0)
                    buffer[iterator] = 1;
                else
                    buffer[iterator] = -1;
            }

            // Find the start of a Sub-frame
            if (iterator == samplesForPreamble - 1) {
                travelTimeQue.assign(buffer, buffer + samplesForPreamble);
                subframeStart = findSubframeStart(travelTimeQue);
                result = codePhaseMs + subframeStart;
            }

            // Find updated subframe start and pass it as a result for further nav
            // calculations
            if (iterator > samplesForPreamble && iterator % 500 == 0) {
                int start = findSubframeStart(travelTimeQue);
                result = codePhaseMs + start;
            }

            // Find ephemerides. Min 5 subframes is required

            if (iterator == (subframeStart + 1500 * 20 - 1)) {
                // Prepare 5 sub frames worth of data in bit format
                std::vector<int> navBits;
                int sum{ 0 };
                int counter{ 0 };
                for (int i = subframeStart - 20; i < subframeStart + 1500 * 20; i++) {
                    sum += buffer[i];
                    counter++;
                    if (counter == 20) {
                        if (sum > 0)
                            navBits.push_back(1);
                        else
                            navBits.push_back(0);
                        counter = 0;
                        sum = 0;
                    }
                }

                Ephemeris ephResults(navBits, channel);
                auto size = sizeof(Ephemeris);
                auto pmt = pmt::make_blob(reinterpret_cast<void*>(&ephResults), size);

                message_port_pub(pmt::string_to_symbol("ephemeris"), pmt);
            }


            iterator++;
        }
        // std::cout << "resuolt: " << result << std::endl;
        out[j] = result;
    }


    return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
