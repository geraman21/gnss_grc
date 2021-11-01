/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "tracking_ff_impl.h"
#include "acqResults.h"
#include "generate_l1_ca.h"
#include "helper-functions.h"
#include <algorithm> // std::transform std::for_each
#include <gnuradio/io_signature.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <math.h>
#include <numeric>

namespace gr {
namespace gnss {

using input_type = gr_complex;
using output_type = float;
tracking_ff::sptr tracking_ff::make(int _channelNum, float _sampleFreq, float pll_nbw, float pll_dr,
                                    float pll_lg, float dll_nbw, float dll_dr, float dll_lg) {
  return gnuradio::make_block_sptr<tracking_ff_impl>(_channelNum, _sampleFreq, pll_nbw, pll_dr,
                                                     pll_lg, dll_nbw, dll_dr, dll_lg);
}

/*
 * The private constructor
 */

tracking_ff_impl::tracking_ff_impl(int _channelNum, float _sampleFreq, float pll_nbw, float pll_dr,
                                   float pll_lg, float dll_nbw, float dll_dr, float dll_lg)
    : gr::sync_block(
          "tracking_ff",
          gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
      channelNum{_channelNum}, samplePeriod{1 / _sampleFreq}, sampleFreq{_sampleFreq},
      pllNoiseBandwidth{pll_nbw}, pllDampingRatio{pll_dr},
      loopGainCarr(pll_lg), dllNoiseBandwidth{dll_nbw}, dllDampingRatio{dll_dr}, loopGainCode{
                                                                                     dll_lg} {
  // channel = new Channel(21, 9547426.34201050, 13404, 'T');
  message_port_register_in(pmt::string_to_symbol("acquisition"));
  message_port_register_out(pmt::string_to_symbol("data_vector"));
  set_msg_handler(pmt::mp("acquisition"), [this](const pmt::pmt_t &msg) {
    auto msg_key = pmt::car(msg);
    auto msg_val = pmt::cdr(msg);
    AcqResults acqResult = *(reinterpret_cast<const AcqResults *>(pmt::blob_data(msg_val)));
    if (pmt::symbol_to_string(msg_key) == "acq_result" && acqResult.channelNumber == channelNum) {
      PRN = acqResult.PRN;
      startReaquisition();
    } else if (pmt::symbol_to_string(msg_key) == "acq_start") {
      if (PRN == acqResult.PRN) {
        if (acqResult.peakMetric > 0)
          handleAcqStart(acqResult);
        else
          startReaquisition();
      }
    }
    // else if (pmt::symbol_to_string(msg_key) == "acq_restart") {
    //   if (PRN == acqResult.PRN)
    //     haltTracking();
    // }
  });
  longSignal.reserve(11000 * samplesPerCode);
  paddedCaTable.reserve(33);
  makePaddedCaTable(paddedCaTable);
  codePhaseStep = codeFreq * samplePeriod;
  samplesPerCode = round(sampleFreq / (codeFreqBasis / codeLength));
  blksize = ceil((codeLength - remCodePhase) / codePhaseStep);
  calcloopCoef(tau1carr, tau2carr, pllNoiseBandwidth, pllDampingRatio, loopGainCarr, PDI);
  calcloopCoef(tau1code, tau2code, dllNoiseBandwidth, dllDampingRatio, loopGainCode, PDI);
}

/*
 * Our virtual destructor.
 */
tracking_ff_impl::~tracking_ff_impl() {}
void tracking_ff_impl::handleAcqStart(AcqResults acqResult) {
  std::cout << std::endl
            << "Freq:  " << acqResult.carrFreq << "    Phase:  " << acqResult.codePhase
            << std::endl;
  codeFreq = codeFreqBasis;
  codePhaseStep = codeFreq * samplePeriod;
  blksize = ceil(codeLength / codePhaseStep);
  restartAcquisition = false;
  unsigned long totalSamplesFromStart = totalSamples - acqResult.codePhase;
  receivedCodePhase = blksize - (totalSamplesFromStart % blksize);
  codePhase = receivedCodePhase;
  carrFreq = acqResult.carrFreq;
  carrFreqBasis = acqResult.carrFreq;
  PRN = acqResult.PRN;
}

void tracking_ff_impl::startReaquisition() {
  restartAcquisition = true;
  collectSamples = true;
  doTracking = false;
  totalSamples = 0;
}
void tracking_ff_impl::reset() {
  msCount = 0;
  signChangeCount = 0;
  remCodePhase = 0.0;
  remCarrPhase = 0.0;
  oldCarrNco = 0.0;
  oldCarrError = 0.0;
  oldCodeNco = 0.0;
  oldCodeError = 0.0;
  carrFreq = 0.0;
  codeFreq = codeFreqBasis;
  iterator = 0;
  Q_E = I_E = Q_P = I_P = Q_L = I_L = 0.0;
}

void tracking_ff_impl::haltTracking() {
  restartAcquisition = false;
  collectSamples = false;
  doTracking = false;
  totalSamples = 0;
  PRN = 0;
  reset();
}

int tracking_ff_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
  output_type *out = reinterpret_cast<output_type *>(output_items[0]);

  // Restart the channel if output data bitrate is above 50hz
  // Allow 200ms for channel to stabilize
  if (msCount >= msForQualityCheck + msToStabilize) {
    double ratio = positiveCorrCount > negativeCorrCount
                       ? positiveCorrCount * 1.0 / negativeCorrCount
                       : negativeCorrCount * 1.0 / positiveCorrCount;
    if (signChangeCount > msForQualityCheck / 20 + 5 || signChangeCount < 10 || ratio > 3) {
      std::cout << "PRN:  " << PRN << "  Quality Check failed:   " << signChangeCount << "    "
                << ratio << std::endl;
      startReaquisition();
      trackingLocked = false;
    } else {
      trackingLocked = true;
    }
    // std::cout << "Quality Results:   " << signChangeCount << "     out of    "
    //           << msForQualityCheck / 20 << std::endl;
    msCount = 0;
    signChangeCount = 0;
    positiveCorrCount = 0;
    negativeCorrCount = 0;
  }

  // Declare Early Late and Prompt code variables and their starting points
  float tStartEarly = remCodePhase - earlyLateSpc;
  float tStartLate = remCodePhase + earlyLateSpc;
  float tStartPrompt = remCodePhase;
  float tEndPrompt = blksize * codePhaseStep + remCodePhase;
  for (int i = 0; i < noutput_items; i++) {
    absSampleCount++;
    if (restartAcquisition) {
      totalSamples++;
    }
    if (collectSamples) {
      if (longSignal.size() < 11 * samplesPerCode) {
        longSignal.push_back(in[i]);
      } else {
        auto size = sizeof(float) * longSignal.size();
        auto pmt = pmt::make_blob(longSignal.data(), size);
        message_port_pub(pmt::mp("data_vector"), pmt::cons(pmt::from_long(PRN), pmt));
        longSignal.clear();
        longSignal.reserve(11 * samplesPerCode);
        collectSamples = false;
      }
    }

    if (!doTracking && PRN != 0 && !restartAcquisition) {
      if (codePhase > 0)
        codePhase--;
      if (codePhase == 0) {
        reset();
        doTracking = true;
        // std::cout << "PRN    " << PRN << "   alligned tracking started" << std::endl;
      }
    }

    if (doTracking) {
      float iteratorStep = codePhaseStep * iterator;
      if (isnan(iteratorStep))
        continue;
      // Generate Early CA Code.
      float earlyCode{}, lateCode{}, promptCode{};

      earlyCode = paddedCaTable.at(PRN).at(std::ceil(tStartEarly + iteratorStep));

      // Generate Late CA Code.
      lateCode = paddedCaTable.at(PRN).at(std::ceil(tStartLate + iteratorStep));

      // Generate Prompt CA Code.
      promptCode = paddedCaTable.at(PRN).at(std::ceil(tStartPrompt + iteratorStep));

      float trigArg = (carrFreq * 2 * M_PI * (iterator * samplePeriod)) + remCarrPhase;

      if (iterator == 0) {
        a = carrFreq * 2 * M_PI * samplePeriod;
        b = remCarrPhase;
        sina = sinf(a);
        cosa = cosf(a);
        resSin = sinf(b);
        resCos = cosf(b);
      }
      float newResCos, newResSin;
      newResCos = cosa * resCos - sina * resSin;
      newResSin = sina * resCos + cosa * resSin;
      resCos = newResCos;
      resSin = newResSin;

      gr_complex qSignal = in[i] * resCos;
      gr_complex iSignal = in[i] * resSin;

      Q_E += earlyCode * qSignal;
      I_E += earlyCode * iSignal;
      Q_P += promptCode * qSignal;
      I_P += promptCode * iSignal;
      Q_L += lateCode * qSignal;
      I_L += lateCode * iSignal;

      // When 1 ms of data is processed update code and carr remaining phase values
      // and reset iterator else incr iterator
      if (iterator == blksize - 1) {
        std::complex<float> currentOutput = I_P + Q_P;
        // quality check whether we receive a real 50hz signal, allow 200ms for channel to stabilize
        if (signbit(prevOutput.real()) != signbit(currentOutput.real()) &&
            msCount > msToStabilize) {
          signChangeCount++;
        }
        // Update output value to I_P

        currentOutput.real() > 0 ? positiveCorrCount++ : negativeCorrCount++;

        output = currentOutput.real();
        remCarrPhase =
            fmodf((carrFreq * 2 * M_PI * ((blksize)*samplePeriod) + remCarrPhase), (2 * M_PI));

        // Update remaining Code Phase once per ms
        remCodePhase = tEndPrompt - 1023.0;

        tag_t tag;
        tag.offset = this->nitems_written(0) + i;
        tag.key = pmt::mp(std::to_string(PRN));
        this->add_item_tag(0, tag);

        //  Find PLL error and update carrier NCO
        //  Implement carrier loop discriminator (phase detector)
        auto cross =
            currentOutput.real() * prevOutput.imag() - prevOutput.real() * currentOutput.imag();
        auto dot = std::abs(currentOutput.real() * prevOutput.real() +
                            prevOutput.imag() * currentOutput.imag());

        if (dot < 0)
          dot = -dot;

        float carrError = atan2(cross, dot) / (2.0 * M_PI);

        // Implement carrier loop filter and generate NCO command

        float carrNco = oldCarrNco + tau1carr * (carrError - oldCarrError) + carrError * tau2carr;
        oldCarrNco = carrNco;
        oldCarrError = carrError;
        //  Modify carrier freq based on NCO command
        carrFreq = carrFreqBasis + carrNco;

        float sqrtEarly = std::abs(I_E + Q_E);
        float sqrtLate = std::abs(I_L + Q_L);
        float codeError = (sqrtEarly - sqrtLate) / (sqrtEarly + sqrtLate);

        //  Implement code loop filter and generate NCO command
        float codeNco = oldCodeNco + tau1code * (codeError - oldCodeError) + codeError * tau2code;
        oldCodeNco = codeNco;
        oldCodeError = codeError;

        //  Modify code freq based on NCO command and update codePhaseStep
        codeFreq = chippingRate - codeNco;
        codePhaseStep = codeFreq * samplePeriod;
        // update blksize
        blksize = std::ceil((codeLength - remCodePhase) / codePhaseStep);

        // if (test > 10000 && test < 10100) {
        //   std::cout << "    " << blksize << std::endl;
        // }
        // test++;

        // Reset early late and prompt correlation results and set iterator to 0
        Q_E = I_E = Q_P = I_P = Q_L = I_L = 0;
        prevOutput = currentOutput;
        iterator = 0;
        msCount++;
      } else {
        iterator++;
      }
    }
    out[i] = output && trackingLocked ? output : 0;
    output = 0;
  }

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
