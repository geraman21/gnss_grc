/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_TRACKING_FF_IMPL_H
#define INCLUDED_GNSS_TRACKING_FF_IMPL_H

#include <gnss/tracking_ff.h>
#include <vector>
#include "channel.h"
#include "helper-functions.h"
#include "generate_l1_ca.h"

namespace gr {
  namespace gnss {

    class tracking_ff_impl : public tracking_ff
    {
     private:
      // Nothing to declare in this block.
      int codePhase;
      float carrFreq;
      float carrFreqBasis;
      Channel *channel {nullptr};
      std::vector<int> caCode;
      float codeFreq = 1.023e6;
      float oldCarrNco = 0.0;
      float oldCarrError = 0.0;
      float sampleFreq = 38.192e6;
      float dllCorrelatorSpacing = 0.5;
      float I_P = 0;
      int codeLength = 1023;
      std::vector<float> buffer;
      float remCarrPhase = 0;
      float remCodePhase = 0;
      float PDI = 0.001;
      float oldCodeNco = 0;
      float oldCodeError = 0;
      float earlyLateSpc = 0.5;
      float chippingRate = 1.023e6;
      short pllNoiseBandwidth = 25;
      short dllNoiseBandwidth = 2;
      float pllDampingRatio = 0.7;
      float dllDampingRatio = 0.7;
      float loopGainCarr = 0.25;
      float loopGainCode = 1;

      float tau1carr, tau2carr;
      float tau1code, tau2code;
      

     public:
      tracking_ff_impl();
      ~tracking_ff_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */

