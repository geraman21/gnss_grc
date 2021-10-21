/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_ACQUISITION_IMPL_H
#define INCLUDED_GNSS_ACQUISITION_IMPL_H

#include "acqResults.h"
#include "cmath"
#include <gnss/acquisition.h>
#include <vector>

namespace gr {
namespace gnss {

class acquisition_impl : public acquisition {
private:
  std::vector<std::vector<std::complex<float>>> complexCaTable;
  int channelNum{};
  bool doColdStart = true;
  float sampleFreq;
  double ts;
  float IF;
  unsigned int iterator = 0;
  float codeFreqBasis = 1023000;
  float codeLength = 1023;
  float samplesPerCode;
  int numberOfFrqBins = 29;
  std::vector<std::vector<std::complex<float>>> caCodesTable;
  std::vector<float> longSignal;
  std::vector<AcqResults> acqResults;

public:
  acquisition_impl(float a_sampleFreq, float im_freq, int a_channelNum);
  acquisition_impl();
  ~acquisition_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_ACQUISITION_IMPL_H */
