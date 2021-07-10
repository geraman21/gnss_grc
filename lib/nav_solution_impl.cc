/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ephemeris.h"
#include "helper-functions.h"
#include "nav_solution_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {

using input_type = float;
using output_type = float;
nav_solution::sptr nav_solution::make()
{
    return gnuradio::make_block_sptr<nav_solution_impl>();
}


/*
 * The private constructor
 */
nav_solution_impl::nav_solution_impl()
    : gr::block("nav_solution",
                gr::io_signature::make(
                    1 /* min inputs */, 8 /* max inputs */, sizeof(input_type)),
                gr::io_signature::make(
                    1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)))
{
    channels.assign(8, Ephemeris());
    message_port_register_in(pmt::string_to_symbol("ephemeris"));
    set_msg_handler(pmt::mp("ephemeris"), [this](const pmt::pmt_t& msg) {
        Ephemeris data_object(*(reinterpret_cast<const Ephemeris*>(pmt::blob_data(msg))));
        // data_object.printEphemeris();
        channels.at(data_object.channelNumber) = data_object;
    });
}

/*
 * Our virtual destructor.
 */
nav_solution_impl::~nav_solution_impl() {}

void nav_solution_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    // #pragma message( \
//     "implement a forecast that fills in how many items on each input you need to produce noutput_items and remove this warning")
    /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
}

int nav_solution_impl::general_work(int noutput_items,
                                    gr_vector_int& ninput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items)
{
    const input_type* in = reinterpret_cast<const input_type*>(input_items[0]);
    output_type* out = reinterpret_cast<output_type*>(output_items[0]);

    // if (test < 10) {
    //     test++;
    //     std::cout << "ninput_items: " << input_items.size() << std::endl;
    // }

    if (initDelay > 0) {
        initDelay -= noutput_items;
    }


    // Do <+signal processing+>
    for (int i = 0; i < ninput_items[0]; i++) {

        if (initDelay < 0 && i == (noutput_items + initDelay - 1)) {
            initDelay = 0;
        }

        if (initDelay == 0 && iterator % 500 == 0) {
            pseudoRanges = getPseudoRanges(input_items, i, startOffset, c);
            if (test < 50) {
                std::cout << "PSEUDO RANGES" << std::endl;
                for (auto range : pseudoRanges) {
                    std::cout << range << ",  ";
                }
                std::cout << std::endl;
            }
            test++;
        }

        out[i] = in[i];
        iterator++;
    }

    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
