/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_NAV_SOLUTION_IMPL_H
#define INCLUDED_GNSS_NAV_SOLUTION_IMPL_H

#include "ephemeris.h"
#include "sat-position.h"
#include <deque>
#include <gnss/nav_solution.h>
#include <vector>

namespace gr {
namespace gnss {

class nav_solution_impl : public nav_solution {
private:
  std::vector<Ephemeris> ephemerides;
  float sampleFreq;
  int decimation;
  bool firstRun = true;
  int numberOfChannels{};
  bool startNavigation = true;
  std::vector<bool> gatherNavBits;
  std::vector<int> subframeStart;
  std::vector<int> liveSubframeStart;
  std::vector<int> PRN;
  double live_TOW{};
  double temp_TOW{};
  int samplesForSubframeStart = 14000;
  std::vector<std::deque<int>> navBits;
  std::vector<std::vector<tag_t>> tags;
  std::vector<double> receivedTime;
  std::vector<uint64_t> iterator;
  // The speed of light, [m/s]
  long int c = 299792458;
  double startOffset = 68.802;
  std::vector<double> pseudoRanges;

public:
  void restartSubframeStartSearch();
  void getEphemerisBits(int startInd, std::deque<int> &source, std::vector<int> &res);
  bool gatherBits();
  nav_solution_impl(float _sampleFreq, int _updateRate);
  ~nav_solution_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_SOLUTION_IMPL_H */
