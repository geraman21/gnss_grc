/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "decoding_test_impl.h"
#include "helper-functions.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
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
    corrResult.reserve(14159);
    int reversePreambleShort[]{ 1, 1, 0, 1, 0, 0, 0, 1 };

    for (int i = 0; i < 8; i++) {
        for (int n = i * 20; n < i + 20; n++) {
            reversePreambleShort[i] ? reversePreamble[n] = 1 : reversePreamble[n] = -1;
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

    // Do <+signal processing+>
    for (int i = 0; i < noutput_items; i++) {
        buffer[iterator] = in[i] > 0 ? 1 : -1;

        out[i] = in[i];
        if (iterator == 13999) {
            convolve(&corrResult, buffer, reversePreamble, 14000, 160);
            std::vector<int>::iterator it = std::find_if(
                corrResult.begin(), corrResult.end(), [](int a) { return a == 160; });
            std::string msg = "correlation succes at index: ";
            msg += std::to_string(it - corrResult.begin());
            printMessage(msg);
        }
    }


    // Tell runtime system how many output items we produced.
    iterator++;
    return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
