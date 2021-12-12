/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "nav_decoding_impl.h"
#include "ephemeris.h"
#include "helper-functions.h"
#include <algorithm>
#include <cmath>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <string>
#include <vector>

namespace gr {
namespace gnss {

using input_type = float;
using output_type = float;
nav_decoding::sptr nav_decoding::make(int channelNum, float _sampleFreq) {
  return gnuradio::make_block_sptr<nav_decoding_impl>(channelNum, _sampleFreq);
}

/*
 * The private constructor
 */
nav_decoding_impl::nav_decoding_impl(int channelNum, float _sampleFreq)
    : gr::sync_block(
          "nav_decoding",
          gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
      sampleFreq{_sampleFreq} {
  set_tag_propagation_policy(TPP_ONE_TO_ONE);
  channel = channelNum;
  message_port_register_out(pmt::string_to_symbol("nav_bits"));
  samplesForPreamble = 14000;
}

/*
 * Our virtual destructor.
 */
nav_decoding_impl::~nav_decoding_impl() {}

void nav_decoding_impl::restartDataExtraction() {
  iterator = 0;
  subframeStart = 0;
}

int nav_decoding_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
  output_type *out = reinterpret_cast<output_type *>(output_items[0]);

  const uint64_t nread = this->nitems_read(0); // number of items read on port 0
  const size_t ninput_items = noutput_items;
  // read all tags associated with port 0 for items in this work function
  tags.clear();
  this->get_tags_in_range(tags, 0, nread, nread + ninput_items);

  for (int i = 0; i < noutput_items; i++) {

    if (tags.size() == ninput_items) {
      int receivedPRN{};
      try {
        receivedPRN = std::stoi(pmt::symbol_to_string(tags.at(i).key));
      } catch (const std::exception &e) {
        std::cout << "Bad PRN received from Tag in nav_decoding_impl.cc" << std::endl;
        std::cerr << e.what() << '\n';
      }
      if (receivedPRN != 0 && PRN != receivedPRN && in[i] != 0) {
        restartDataExtraction();
        PRN = receivedPRN;
        gatherNavBits = true;
      }
    }

    if (gatherNavBits) {
      if (in[i] == 0) {
        restartDataExtraction();
        continue;
      }

      if (iterator < subframeStart + 1500 * 20 - 1) {

        buffer[iterator] = in[i];

        // Find the start of a Sub-frame to generate correct nav-bits
        if (iterator == samplesForPreamble - 1) {
          std::deque<int> temp(buffer, buffer + samplesForPreamble);
          auto [s, t] = findSubframeStart(temp);
          subframeStart = s;
          if (subframeStart == 0) {
            restartDataExtraction();
          }
        }
      }
      if (iterator == (subframeStart + 1500 * 20 - 1)) {
        // Prepare 5 sub frames worth of data in bit format
        std::vector<int> navBits;
        int sum{0};
        int counter{0};
        for (int i = subframeStart - 20; i < subframeStart + 1500 * 20; i++) {
          sum += buffer[i];
          counter++;
          if (counter == 20) {
            sum > 0 ? navBits.push_back(1) : navBits.push_back(0);
            counter = 0;
            sum = 0;
          }
        }
        auto size = sizeof(int) * navBits.size();
        auto pmt = pmt::make_blob(navBits.data(), size);
        message_port_pub(pmt::string_to_symbol("nav_bits"),
                         pmt::cons(pmt::from_long(channel), pmt));

        std::cout << "Nav bits for PRN   " << PRN << "sent to nav_solution" << std::endl;
        gatherNavBits = false;
      }
      if (gatherNavBits)
        iterator++;
    }
    out[i] = in[i];
  }
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
