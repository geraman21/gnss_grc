/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_TRACKING_FF_IMPL_H
#define INCLUDED_GNSS_TRACKING_FF_IMPL_H

#include "generate_l1_ca.h"
#include "helper-functions.h"
#include <complex>
#include <deque>
#include <gnss/tracking_ff.h>
#include <vector>

namespace gr {
namespace gnss {

class tracking_ff_impl : public tracking_ff {
private:
  unsigned long int totalSamples{};
  bool doTracking = false;
  bool restartAcquisition = false;
  bool collectSamples = false;
  int PRN{};
  float peakMetric{};
  int channelNum;
  int msForQualityCheck = 1000;
  int msToStabilize = 2000;
  int msCount{};
  int bitTransitionCount{};
  int positiveCorrCount{};
  int negativeCorrCount{};
  bool trackingLocked = false;
  // for sin cos calculations
  float a, b, sina, cosa, resSin, resCos;
  std::complex<float> I_E, Q_E, Q_P, I_P, I_L, Q_L;
  int iterator = 0;
  int blksize{};
  int codePhase{};
  int receivedCodePhase{};
  float carrFreq;
  float carrFreqBasis;
  std::vector<std::vector<int>> paddedCaTable;
  std::vector<int> caCode;
  std::vector<std::complex<float>> longSignal;
  int samplesPerCode{};
  float codeFreqBasis = 1023000;
  float codeFreq = 1023000;
  float oldCarrNco = 0.0;
  float oldCarrError = 0.0;
  float samplePeriod;
  float sampleFreq;
  float codePhaseStep = 0;
  float dllCorrelatorSpacing = 0.5;
  float output = 0;
  std::complex<float> prevOutput{};
  int codeLength = 1023;
  float remCarrPhase = 0;
  float remCodePhase = 0;
  float PDI = 0.001;
  float oldCodeNco = 0;
  float oldCodeError = 0;
  float earlyLateSpc = 0.5;
  float chippingRate = 1023000;
  float dllNoiseBandwidth;
  float pllNoiseBandwidth;
  float pllDampingRatio = 0.7;
  float dllDampingRatio = 0.7;
  float loopGainCarr = 0.25;
  float loopGainCode = 1;

  float tau1carr, tau2carr;
  float tau1code, tau2code;
  void reset();
  void handleAcqStart(AcqResults acqResult);
  void startReaquisition();

public:
  tracking_ff_impl(int _channelNum, float _sampleFreq, float pll_nbw, float dll_nbw);
  tracking_ff_impl();
  ~tracking_ff_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */
