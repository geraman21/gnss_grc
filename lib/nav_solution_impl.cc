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
#include <algorithm>
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {

using input_type = float;
using output_type = float;
nav_solution::sptr nav_solution::make(float _sampleFreq, int _updateRate) {
  return gnuradio::make_block_sptr<nav_solution_impl>(_sampleFreq, _updateRate);
}

/*
 * The private constructor
 */
nav_solution_impl::nav_solution_impl(float _sampleFreq, int _updateRate)
    : gr::sync_decimator(
          "nav_solution",
          gr::io_signature::make(1 /* min inputs */, 8 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)),
          1000 / _updateRate),
      sampleFreq(_sampleFreq), decimation{1000 / _updateRate} {

  message_port_register_in(pmt::string_to_symbol("ephemeris"));
  set_msg_handler(pmt::mp("ephemeris"), [this](const pmt::pmt_t &msg) {
    Ephemeris data_object(*(reinterpret_cast<const Ephemeris *>(pmt::blob_data(msg))));
    if (data_object.channelNumber != -1) {
      ephemerides.at(data_object.channelNumber) = data_object;
      +std::cout.precision(12);
      data_object.printEphemeris();
      std::cout << "============================" << std::endl;
    }
  });
}

void nav_solution_impl::restartSubframeStartSearch() {
  navBits.clear();
  navBits.resize(numberOfChannels);
  subframeStart.clear();
  subframeStart.resize(numberOfChannels);
  iterator.clear();
  iterator.resize(numberOfChannels, 0);
  gatherNavBits.clear();
  gatherNavBits.resize(numberOfChannels, true);
}

bool nav_solution_impl::gatherBits() {
  return std::all_of(gatherNavBits.begin(), gatherNavBits.end(), [](bool v) { return v; });
}

void nav_solution_impl::getEphemerisBits(int startInd, std::deque<int> &source,
                                         std::vector<int> &res) {
  if (source.size() < (startInd + 1500 * 20) || startInd < 20) {
    return;
    std::cout << "source size:  " << source.size() << "   startInd:  " << startInd << std::endl;
  }
  int sum{0};
  int counter{0};
  for (int i = startInd - 20; i < startInd + 1500 * 20; i++) {
    sum += source[i];
    counter++;
    if (counter == 20) {
      if (sum > 0)
        res.push_back(1);
      else
        res.push_back(0);
      counter = 0;
      sum = 0;
    }
  }
}

/*
 * Our virtual destructor.
 */
nav_solution_impl::~nav_solution_impl() {}

int nav_solution_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items) {
  if (firstRun) {
    numberOfChannels = input_items.size();
    ephemerides.assign(numberOfChannels, Ephemeris());
    navBits.resize(numberOfChannels);
    gatherNavBits.resize(numberOfChannels, false);
    iterator.resize(numberOfChannels, 0);
    subframeStart.resize(numberOfChannels, 0);
    liveSubframeStart.resize(numberOfChannels, 0);
    receivedTime.resize(numberOfChannels);
    PRN.resize(numberOfChannels, 0);
    firstRun = false;
  }
  tags.clear();
  tags.resize(numberOfChannels);
  const size_t ninput_items = noutput_items * decimation;
  for (int i = 0; i < numberOfChannels; i++) {
    const uint64_t nread = this->nitems_read(i); // number of items read on port i
    this->get_tags_in_range(tags.at(i), i, nread, nread + ninput_items);
  }

  const input_type *in0 = reinterpret_cast<const input_type *>(input_items[0]);
  output_type *out = reinterpret_cast<output_type *>(output_items[0]);

  // std::cout << "Tags size:  " << tags.size() << std::endl;
  for (int j = 0; j < noutput_items; j++) {
    if (live_TOW != 0) {
      live_TOW += 0.5;
    }

    for (int p = 0; p < input_items.size(); p++) {
      if (liveSubframeStart.at(p) != 0) {
        receivedTime.at(p) =
            (double)pmt::to_uint64(tags.at(p).at(j * decimation + liveSubframeStart.at(p)).value) /
            ((double)sampleFreq / 1000.0);
        // std::cout << pmt::to_uint64(tags.at(p).at(j * decimation +
        // liveSubframeStart.at(p)).value)
        //           << "    ";
      }
      // if (startNavigation) {
      //   std::cout << pmt::to_uint64(tags.at(p).at(j * decimation +
      //   liveSubframeStart.at(p)).value)
      //             << "     ";
      // }
    }
    // std::cout << std::endl;

    for (int i = j * decimation; i < j * decimation + decimation; i++) {

      for (int p = 0; p < input_items.size(); p++) {
        const float *in = reinterpret_cast<const input_type *>(input_items[p]);

        int receivedPRN{};
        try {
          receivedPRN = std::stoi(pmt::symbol_to_string(tags.at(p).at(i).key));
        } catch (const std::exception &e) {
          std::cout << "Couldnt extract PRN from Tag in nav_solution_impl.cc" << std::endl;
          std::cerr << e.what() << '\n';
        }
        if (receivedPRN != 0 && PRN.at(p) != receivedPRN) {
          std::cout << "Sub Start search restarted,   old PRN:   " << PRN.at(p)
                    << "    new PRN:   " << receivedPRN << std::endl;
          PRN.at(p) = receivedPRN;
          restartSubframeStartSearch();
          break;
        }

        if (gatherNavBits.at(p)) {

          if (in[i] == 0 && iterator.at(p) < samplesForSubframeStart - 1) {
            restartSubframeStartSearch();
            break;
          }
          navBits.at(p).push_back(in[i]);

          if (iterator.at(p) == samplesForSubframeStart - 1) {
            auto [s, t] = findSubframeStart(navBits.at(p));
            if (s != 0 && t != 0) {
              subframeStart.at(p) = s;
              temp_TOW = t;
              std::cout << "PRN:  " << PRN.at(p) << "    Subframe Start:  " << s
                        << "    TOW:  " << t << "    iterator:  " << iterator.at(p) << std::endl;
            } else {
              std::cout << "Didnt find substart index, Subframe Start search restarted "
                        << std::endl;
              restartSubframeStartSearch();
              break;
            }
          }

          if (iterator.at(p) == (subframeStart.at(p) + 900 * 20 - 1)) {
            liveSubframeStart.at(p) = i % decimation;
            live_TOW = temp_TOW + 12;
            receivedTime.at(p) =
                (double)pmt::to_uint64(tags.at(p).at(i).value) / ((double)sampleFreq / 1000.0);
            gatherNavBits.at(p) = false;
            // std::vector<int> ephBits;
            // getEphemerisBits(subframeStart.at(p), navBits.at(p), ephBits);
            // if (ephBits.size() >= 1501) {
            //   ephemerides.at(p) = Ephemeris(ephBits, p);
            //   if (TOW == 0) {
            //     TOW = ephemerides.at(p).TOW;
            //   }
            //   std::cout << "TOW main:  " << TOW << "    from EPH:  " << ephemerides.at(p).TOW
            //             << std::endl;
            //   gatherNavBits.at(p) = false;
            // } else {
            //   std::cout << "Collecting ephBits failed restarting... " << ephBits.size()
            //             << std::endl;
            //   restartSubframeStartSearch();
            //   break;
            // }
            // std::cout << "Ephemeris Data for PRN:  " << PRN.at(p) << "  ready" << std::endl;
          }
          iterator.at(p)++;
        }
      }
    }

    // for (auto bits : navBits) {
    //   std::cout << bits.size() << "     ";
    // }
    // std::cout << std::endl;
    std::vector<double> active_input_items;
    std::vector<Ephemeris> active_ephemerides;
    for (int p = 0; p < numberOfChannels; p++) {
      if (ephemerides.at(p).channelNumber != -1 && receivedTime.at(p) != 0) {
        active_input_items.push_back(receivedTime.at(p));
        active_ephemerides.push_back(ephemerides.at(p));
      }
    }

    // Ensure that at least 4 channels are available
    startNavigation = true;

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
      // for (int i = 0; i < liveSubframeStart.size(); i++) {
      //   std::cout << liveSubframeStart.at(i) << "  " << active_input_items.at(i) << "      ";
      // }
      // std::cout << std::endl;
      pseudoRanges = getPseudoRanges(active_input_items, startOffset, c);
      std::vector<SatPosition> satPositions(active_input_items.size());
      for (int i = 0; i < active_input_items.size(); i++) {
        satPositions.at(i) = SatPosition(live_TOW, active_ephemerides.at(i));
        pseudoRanges.at(i) = pseudoRanges.at(i) + satPositions.at(i).satClkCorr * c;
        // std::cout << pseudoRanges.at(i) << "     ";
      }
      std::cout << std::endl;

      auto [xyzdt, el, az, DOP] = leastSquarePos(satPositions, pseudoRanges, c);
      auto [latitude, longitude, height] = cart2geo(xyzdt(0), xyzdt(1), xyzdt(2), 5);
      // cout << "xyzdt(0): " << xyzdt(0) << "  xyzdt(1): " << xyzdt(1) << "  xyzdt(2): " <<
      // xyzdt(2)
      //      << endl;
      cout << "latitude: " << latitude << "  longitude: " << longitude << "  height: " << height
           << "     TOW:  " << live_TOW << endl;
      //  std::cout << "xyzdt: " << xyzdt << std::endl
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
    out[j] = 0;
  }

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
