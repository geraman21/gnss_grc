/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "decoding_test_impl.h"
#include "ephemeris.h"
#include "helper-functions.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <algorithm>
#include <string>
#include <vector>


namespace gr {
namespace gnss {

using input_type = float;
using output_type = float;
decoding_test::sptr decoding_test::make(int prn)
{
    return gnuradio::make_block_sptr<decoding_test_impl>(prn);
}

/*
 * The private constructor
 */
decoding_test_impl::decoding_test_impl(int prn)
    : gr::sync_block("decoding_test",
                     gr::io_signature::make(
                         1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                     gr::io_signature::make(
                         1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
      PRN(prn)
{
    message_port_register_out(pmt::string_to_symbol("result"));
    corrResult = std::vector<int>(14159);
    int reversePreambleShort[]{ 1, 1, 0, 1, 0, 0, 0, 1 };

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
decoding_test_impl::~decoding_test_impl() {}

void decoding_test_impl::printMessage(std::string msg)
{
    message_port_pub(pmt::string_to_symbol("result"), pmt::string_to_symbol(msg));
}

int decoding_test_impl::work(int noutput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
    const input_type* in = reinterpret_cast<const input_type*>(input_items[0]);
    output_type* out = reinterpret_cast<output_type*>(output_items[0]);


    for (int i = 0; i < noutput_items; i++) {

        if (in[i] > 0)
            buffer[iterator] = 1;
        else
            buffer[iterator] = -1;

        out[i] = in[i];

        // Find the start of a Sub-frame
        if (iterator == 13999) {
            convolve(&corrResult, buffer, reversePreamble, 14000, 160);

            // while (test < 160) {
            //     std::cout << corrResult.at(test) << ", ";
            //     test++;
            // }

            std::vector<int>::iterator it = corrResult.begin();

            // it = std::find_if(it, corrResult.end(), [](int a) { return a > 153; });

            // std::string msg = "correlation succes at index: ";
            // int dist = std::distance(corrResult.begin(), it);
            // msg += std::to_string(dist);
            // printMessage(msg);

            while (it != corrResult.end()) {
                it++;
                it = std::find_if(
                    it, corrResult.end(), [](int a) { return (std::abs(a) > 153); });
                std::string msg = "correlation succes at index: ";
                int dist = std::distance(corrResult.begin(), it);
                indices.push_back(dist - 159);
            }

            for (int i = indices.size() - 1; i > 0; i--) {
                for (int k = 0; k < i; k++) {
                    if (indices.at(i) - indices.at(k) == 6000) {
                        // Re-read bit vales for preamble verification ==============
                        // Preamble occurrence is verified by checking the parity of
                        // the first two words in the subframe. Now it is assumed that
                        // bit boundaries a known. Therefore the bit values over 20ms are
                        // combined to increase receiver performance for noisy signals.
                        // in Total 62 bits mast be read :
                        // 2 bits from previous subframe are needed for parity checking;
                        // 60 bits for the first two 30bit words (TLM and HOW words).
                        // The index is pointing at the start of TLM word.

                        // Initialize of size 33, add blank element at the start for
                        // better indexing
                        int index = indices.at(k);
                        std::vector<int> bits;
                        int sum{ 0 };
                        int counter{ 0 };
                        for (int i = index - 40; i < index + 60 * 20; i++) {
                            sum += buffer[i];
                            counter++;
                            if (counter == 20) {
                                if (sum > 0)
                                    bits.push_back(1);
                                else
                                    bits.push_back(-1);
                                counter = 0;
                                sum = 0;
                            }
                        }

                        std::vector<int> split_lo(bits.begin(), bits.begin() + 32);
                        std::vector<int> split_hi(bits.begin() + 30, bits.end());
                        split_lo.insert(split_lo.begin(), -10);
                        split_hi.insert(split_hi.begin(), -10);

                        int parity1 = parityCheck(split_lo, index);
                        int parity2 = parityCheck(split_hi, index);

                        std::cout << "parity low: " << parity1 << std::endl;
                        std::cout << "parity high: " << parity2 << std::endl;

                        if (parity1 != 0 && parity2 != 0) {
                            subframeStart = index;
                            parityResult = parity1;
                            break;
                        }
                    }
                }
            }
            std::cout << "subStart: " << subframeStart << std::endl;
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

            std::cout << "Nav Bits size: " << navBits.size() << std::endl;
            Ephemeris* ephResults = new Ephemeris(navBits);

            // std::cout << "weekNumber : " << ephResults->weekNumber << std::endl;
            // std::cout << "accuracy : " << ephResults->accuracy << std::endl;
            // std::cout << "health : " << ephResults->health << std::endl;
            // std::cout << "T_GD : " << ephResults->T_GD << std::endl;
            // std::cout << "IODC : " << ephResults->IODC << std::endl;
            // std::cout << "t_oc : " << ephResults->t_oc << std::endl;
            // std::cout << "a_f2: " << ephResults->a_f2 << std::endl;
            // std::cout << "a_f1 : " << ephResults->a_f1 << std::endl;
            // std::cout << "a_f0 : " << ephResults->a_f0 << std::endl;
            // std::cout << "=========================== case 2 ================"
            //           << std::endl;
            // std::cout << "IODE_sf2 : " << ephResults->IODE_sf2 << std::endl;
            // std::cout << "C_rs : " << ephResults->C_rs << std::endl;
            // std::cout << "deltan : " << ephResults->deltan << std::endl;
            // std::cout << "M_0 : " << ephResults->M_0 << std::endl;
            // std::cout << " C_uc: " << ephResults->C_uc << std::endl;
            // std::cout << "e : " << ephResults->e << std::endl;
            // std::cout << "C_us : " << ephResults->C_us << std::endl;
            // std::cout << " sqrtA: " << ephResults->sqrtA << std::endl;
            // std::cout << "t_oe : " << ephResults->t_oe << std::endl;
            // std::cout << "=========================== case 3 ================"
            //           << std::endl;
            // std::cout << "C_ic : " << ephResults->C_ic << std::endl;
            // std::cout << "omega_0 : " << ephResults->omega_0 << std::endl;
            // std::cout << "C_is : " << ephResults->C_is << std::endl;
            // std::cout << "i_0 : " << ephResults->i_0 << std::endl;
            // std::cout << " C_rc: " << ephResults->C_rc << std::endl;
            // std::cout << "omega : " << ephResults->omega << std::endl;
            // std::cout << "omegaDot : " << ephResults->omegaDot << std::endl;
            // std::cout << " IODE_sf3: " << ephResults->IODE_sf3 << std::endl;
            // std::cout << "iDot : " << ephResults->iDot << std::endl;
        }


        iterator++;
    }


    // Tell runtime system how many output items we produced.

    return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
