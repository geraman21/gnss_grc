/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "nav_solution_impl.h"
#include "ephemeris.h"
#include "geo-functions.h"
#include "helper-functions.h"
#include "sat-position.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {

using input_type = float;
using output_type = float;
nav_solution::sptr nav_solution::make() { return gnuradio::make_block_sptr<nav_solution_impl>(); }

/*
 * The private constructor
 */
nav_solution_impl::nav_solution_impl()
    : gr::sync_block(
          "nav_solution",
          gr::io_signature::make(1 /* min inputs */, 8 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))) {
  ephemerides.assign(8, Ephemeris());
  message_port_register_in(pmt::string_to_symbol("ephemeris"));
  set_msg_handler(pmt::mp("ephemeris"), [this](const pmt::pmt_t &msg) {
    Ephemeris data_object(*(reinterpret_cast<const Ephemeris *>(pmt::blob_data(msg))));
    if (data_object.channelNumber != -1) {
      ephemerides.at(data_object.channelNumber) = data_object;
    }
  });
}

/*
 * Our virtual destructor.
 */
nav_solution_impl::~nav_solution_impl() {}

int nav_solution_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items) {
  const input_type *in0 = reinterpret_cast<const input_type *>(input_items[0]);
  output_type *out = reinterpret_cast<output_type *>(output_items[0]);

  for (int i = 0; i < noutput_items; i++) {

    // Ensure that at least 4 channels are available

    startNavigation = true;
    int activeChannels{};
    for (int p = 0; p < input_items.size(); p++) {
      const input_type *in = reinterpret_cast<const input_type *>(input_items[p]);
      if (in[i] != 0 && ephemerides.at(p).channelNumber != -1)
        activeChannels++;
    }
    if (activeChannels < 4) {
      startNavigation = false;
      test = 0;
    } else {
      if (test == 0) {
        std::cout << "Active satellites:   " << activeChannels
                  << "  -  Navigation Solution in process" << std::endl;
        test++;
      }
    }

    if (startNavigation) {

      pseudoRanges = getPseudoRanges(input_items, i, startOffset, c);

      std::vector<SatPosition> satPositions(input_items.size());
      bool startNavCalculations = true;
      for (int i = 0; i < input_items.size(); i++) {
        satPositions.at(i) = SatPosition(ephemerides.at(i).TOW + iterator * 0.5, ephemerides.at(i));
        pseudoRanges.at(i) = pseudoRanges.at(i) + satPositions.at(i).satClkCorr * c;
      }

      auto [xyzdt, el, az, DOP] = leastSquarePos(satPositions, pseudoRanges, c);
      auto [latitude, longitude, height] = cart2geo(xyzdt(0), xyzdt(1), xyzdt(2), 5);
      cout << "xyzdt(0): " << xyzdt(0) << "  xyzdt(1): " << xyzdt(1) << "  xyzdt(2): " << xyzdt(2)
           << endl;
      // cout << "latitude: " << latitude << "  longitude: " << longitude << "  height: " << height
      // << endl; std::cout << "xyzdt: " << xyzdt << std::endl
      //           << std::endl;
      // std::cout << "El: [ ";
      // for (auto i : el)
      // {
      //     std::cout << i << ", ";
      // }
      // std::cout << "]" << std::endl;

      // std::cout << "Az: [ ";
      // for (auto i : az)
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
