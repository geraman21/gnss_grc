/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_CHANNEL_STARTER_IMPL_H
#define INCLUDED_GNSS_CHANNEL_STARTER_IMPL_H

#include <complex>
#include <gnss/channel_starter.h>
#include <vector>

namespace gr {
namespace gnss {

class channel_starter_impl : public channel_starter {
private:
  int PRN;
  int attemptsNum;
  std::vector<int> attemptsLeft;
  float sampleFreq;
  float ts;
  float codeFreqBasis = 1023000;
  float codeLength = 1023;
  float samplesPerCode;
  std::vector<std::complex<float>> complexCaVector;
  std::vector<float> longSignal;

public:
  channel_starter_impl(int attempts, float s_sampleFreq);
  ~channel_starter_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_CHANNEL_STARTER_IMPL_H */
