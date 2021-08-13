/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_ACQUISITION_IMPL_H
#define INCLUDED_GNSS_ACQUISITION_IMPL_H

#include <gnss/acquisition.h>
#include <vector>
#include <gnuradio/fft/fft.h>
#include "cmath"

namespace gr
{
  namespace gnss
  {

    class acquisition_impl : public acquisition
    {
    private:
      bool doColdStart = true;
      float sampleFreq;
      float ts;
      unsigned int iterator = 0;
      float codeFreqBasis = 1023000;
      float codeLength = 1023;
      float samplesPerCode;
      int numberOfFrqBins = 29;
      float IF = 9.548e6;
      std::vector<int> frqBins;
      std::vector<std::vector<int>> caCodesTable;
      gr::fft::fft_real_fwd p1_fft_real;
      gr::fft::fft_complex_fwd p1_fft;
      gr::fft::fft_complex_rev p1_fft_rev;
      std::vector<float> lognSignal;
      std::vector<std::vector<float>> results;

    public:
      acquisition_impl(float a_sampleFreq);
      acquisition_impl();
      ~acquisition_impl();

      // Where all the action really happens
      int work(int noutput_items, gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_ACQUISITION_IMPL_H */
