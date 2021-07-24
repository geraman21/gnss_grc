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
#include "sat-position.h"
#include "geo-functions.h"

namespace gr
{
    namespace gnss
    {

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
            : gr::sync_block("nav_solution",
                             gr::io_signature::make(
                                 1 /* min inputs */, 8 /* max inputs */, sizeof(input_type)),
                             gr::io_signature::make(
                                 1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)))
        {
            ephemerides.assign(8, Ephemeris());
            message_port_register_in(pmt::string_to_symbol("ephemeris"));
            set_msg_handler(pmt::mp("ephemeris"), [this](const pmt::pmt_t &msg)
                            {
                                Ephemeris data_object(*(reinterpret_cast<const Ephemeris *>(pmt::blob_data(msg))));
                                // data_object.printEphemeris();
                                // std::cout << "===================" << std::endl;
                                ephemerides.at(data_object.channelNumber) = data_object;
                            });
        }

        /*
 * Our virtual destructor.
 */
        nav_solution_impl::~nav_solution_impl() {}

        int nav_solution_impl::work(int noutput_items,
                                    gr_vector_const_void_star &input_items,
                                    gr_vector_void_star &output_items)
        {
            const input_type *in0 = reinterpret_cast<const input_type *>(input_items[0]);
            const input_type *in1 = reinterpret_cast<const input_type *>(input_items[1]);
            const input_type *in2 = reinterpret_cast<const input_type *>(input_items[2]);
            output_type *out = reinterpret_cast<output_type *>(output_items[0]);

            for (int i = 0; i < noutput_items; i++)
            {
                if (!startNavigation && in0[i] != 0 && in1[i] != 0 && in2[i] != 0)
                    startNavigation = true;
                if (in0[i] != 0 && in1[i] != 0 && in2[i] != 0)
                {

                    pseudoRanges = getPseudoRanges(input_items, i, startOffset, c);
                    // std::cout << pseudoRanges.at(0) << "   " << pseudoRanges.at(1) << "     " << pseudoRanges.at(2) << std::endl;

                    std::vector<SatPosition> satPositions(input_items.size());
                    bool startNavCalculations = true;
                    for (int i = 0; i < input_items.size(); i++)
                    {
                        satPositions.at(i) = SatPosition(ephemerides.at(i).TOW + iterator * 0.5, ephemerides.at(i));
                        pseudoRanges.at(i) = pseudoRanges.at(i) + satPositions.at(i).satClkCorr * c;
                        if (!satPositions.at(i).isActive)
                        {
                            startNavCalculations = false;
                        }
                        // else
                        //     std::cout << satPositions.at(i).pos1 << "  " << satPositions.at(i).pos2 << "  " << satPositions.at(i).pos3 << std::endl;
                    }
                    if (startNavCalculations)
                        auto [xyzdt, el, az, DOP] = leastSquarePos(satPositions, pseudoRanges, c);

                    // std::cout << "xyzdt: " << xyzdt << std::endl
                    //           << std::endl;
                    // std::cout << "El: [ ";
                    // for (auto i : el)
                    // {
                    //     std::cout << i << ", ";
                    // }
                    // std::cout << "]" << std::endl;

                    // std::cout << "Az: [ ";
                    // for (auto i : el)
                    // {
                    //     std::cout << i << ", ";
                    // }
                    // std::cout << "]" << std::endl;

                    // std::cout << "DOP: [ ";
                    // for (auto i : DOP)
                    // {
                    //     std::cout << i << ", ";
                    // }
                    // std::cout << "]" << std::endl;

                    // std::cout << std::endl
                    //           << "========================================" << std::endl
                    //           << std::endl;

                    iterator++;
                }
                out[i] = in0[i];
            }

            // Tell runtime system how many output items we produced.
            return noutput_items;
        }

    } /* namespace gnss */
} /* namespace gr */
