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

using input_type = double;
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
      std::cout << "============================" << std::endl;
      data_object.printEphemeris();
      std::cout << "============================" << std::endl;
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
    std::cout.precision(12);
    startNavigation = true;
    std::vector<double> active_input_items;
    std::vector<Ephemeris> active_ephemerides;
    std::vector<double> towOffsets;
    for (int p = 0; p < input_items.size(); p++) {
      const double *in = reinterpret_cast<const input_type *>(input_items[p]);
      tags.clear();
      this->get_tags_in_range(tags, p, nread, nread + ninput_items, pmt::mp("towoffset"));

      if (in[i] != 0 && ephemerides.at(p).channelNumber != -1) {
        active_input_items.push_back(in[i]);
        active_ephemerides.push_back(ephemerides.at(p));
        towOffsets.push_back(pmt::to_double(tags.at(i).value));
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
    // if (navTest == 0) {
    //   vector<SatPosition> satPos{
    //       SatPosition(710590.946167625, -20534747.0669515, 16565299.3863555),
    //       SatPosition(-18853767.1676574, -12561495.5559795, 14004713.975897),
    //       SatPosition(-10834993.1792765, -10565346.9108378, 21612853.8879797),
    //       SatPosition(-8526254.76971803, -15275511.9944097, 20120992.0587014)};
    //   vector<double> pseudoRanges{20767164.2372144, 22165191.9581342, 21321286.6633887,
    //                               21040638.9270996};

    //   auto [xyzdt, el, az, DOP] = leastSquarePos(satPos, pseudoRanges, 299792458);
    //   cout << "xyzdt(0): " << xyzdt(0) << "  xyzdt(1): " << xyzdt(1) << "  xyzdt(2): " <<
    //   xyzdt(2)
    //        << endl;

    //   navTest++;
    // }
    if (startNavigation) {
      pseudoRanges = getPseudoRanges(active_input_items, startOffset, c);
      std::vector<SatPosition> satPositions(active_input_items.size());
      for (int i = 0; i < active_input_items.size(); i++) {
        double transmitTime = active_ephemerides.at(i).TOW * 1.0 + towOffsets.at(i) * 1.0;
        satPositions.at(i) = SatPosition(transmitTime, active_ephemerides.at(i));
        pseudoRanges.at(i) = pseudoRanges.at(i) + satPositions.at(i).satClkCorr * c;
      }
      std::cout << std::endl;

      auto [xyzdt, el, az, DOP] = leastSquarePos(satPositions, pseudoRanges, c);
      auto [latitude, longitude, height] = cart2geo(xyzdt(0), xyzdt(1), xyzdt(2), 5);
      cout << "xyzdt(0): " << xyzdt(0) << "  xyzdt(1): " << xyzdt(1) << "  xyzdt(2): " << xyzdt(2)
           << endl;
      // cout << "latitude: " << latitude << "  longitude: " << longitude << "  height: " <<
      // height
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
    out[i] = in0[i];
  }

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
