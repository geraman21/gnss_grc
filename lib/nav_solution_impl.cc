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
      restartIterator = true;
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

  const uint64_t nread = this->nitems_read(0); // number of items read on port 0
  const size_t ninput_items = noutput_items;   // assumption for sync block, this can change
  // read all tags associated with port 0 for items in this work function

  // std::cout << "Tags size:  " << tags.size() << std::endl;
  for (int i = 0; i < noutput_items; i++) {

    // Ensure that at least 4 channels are available

    startNavigation = true;
    std::vector<float> active_input_items;
    std::vector<Ephemeris> active_ephemerides;
    std::vector<float> towOffsets;
    for (int p = 0; p < input_items.size(); p++) {
      const float *in = reinterpret_cast<const input_type *>(input_items[p]);
      tags.clear();
      this->get_tags_in_range(tags, p, nread, nread + ninput_items, pmt::mp("towoffset"));

      if (in[i] != 0 && ephemerides.at(p).channelNumber != -1) {
        active_input_items.push_back(in[i]);
        active_ephemerides.push_back(ephemerides.at(p));
        towOffsets.push_back(pmt::to_float(tags.at(i).value));
      }
    }
    if (active_input_items.size() < 4) {
      startNavigation = false;
      test = 0;
    } else {
      if (test == 0) {
        std::cout << "Active satellites:   " << active_input_items.size()
                  << "  -  Navigation Solution in process" << std::endl;
        test++;
      }
    }

    if (startNavigation) {

      pseudoRanges = getPseudoRanges(active_input_items, i, startOffset, c);

      std::vector<SatPosition> satPositions(active_input_items.size());
      for (int i = 0; i < active_input_items.size(); i++) {
        satPositions.at(i) =
            SatPosition(active_ephemerides.at(i).TOW + towOffsets.at(i), active_ephemerides.at(i));
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
    }
    // Update TOW
    for (auto eph : ephemerides) {
      eph.TOW += 0.5;
    }
    out[i] = in0[i];
  }

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
