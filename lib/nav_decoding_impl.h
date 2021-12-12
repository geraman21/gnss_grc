/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_NAV_DECODING_IMPL_H
#define INCLUDED_GNSS_NAV_DECODING_IMPL_H
#include <deque>
#include <gnss/nav_decoding.h>
#include <string>
#include <vector>

namespace gr {
namespace gnss {

class nav_decoding_impl : public nav_decoding {
private:
  int PRN{};
  float sampleFreq;
  int channel;
  uint64_t iterator{0};
  bool gatherNavBits = false;
  int samplesForPreamble;
  int subframeStart = 0;
  int buffer[37000];
  std::vector<tag_t> tags;
  void restartDataExtraction();

public:
  nav_decoding_impl(int channelNum, float _sampleFreq);
  ~nav_decoding_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_DECODING_IMPL_H */
