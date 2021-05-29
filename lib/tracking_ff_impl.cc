/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "tracking_ff_impl.h"
#include <algorithm> // std::transform std::for_each
#include "generate_l1_ca.h"
#include "helper-functions.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <numeric> // std::accumulate


namespace gr {
  namespace gnss {

    using input_type = float;
    using output_type = float;
    tracking_ff::sptr
    tracking_ff::make()
    {
      return gnuradio::make_block_sptr<tracking_ff_impl>(
        );
    }


    /*
     * The private constructor
     */
    tracking_ff_impl::tracking_ff_impl()
      : gr::sync_block("tracking_ff",
              gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
              gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)))
    {
      channel = new Channel(21, 9547426.34201050, 13404, 'T');
      codePhase = channel->codePhase;
      carrFreq = channel->acquiredFreq;
      carrFreqBasis = channel->acquiredFreq;
      caCode = generateCa (channel->prn);
      caCode.insert(caCode.begin(), caCode.back());
      caCode.push_back(caCode.at(1));

      calcloopCoef(tau1carr, tau2carr, pllNoiseBandwidth, pllDampingRatio, loopGainCarr, PDI);
      calcloopCoef(tau1code, tau2code, dllNoiseBandwidth, dllDampingRatio, loopGainCode, PDI);

      delete channel;
    }

    /*
     * Our virtual destructor.
     */
    tracking_ff_impl::~tracking_ff_impl()
    {
    }

    int
    tracking_ff_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const input_type *in = reinterpret_cast<const input_type*>(input_items[0]);
      output_type *out = reinterpret_cast<output_type*>(output_items[0]);

      // Add incoming data into the buffer to work on later
      buffer.insert(buffer.end(), in, in + noutput_items);

      // Align the phase of the signal
      if (codePhase > 0 && buffer.size() > codePhase) {
        buffer.erase(buffer.begin(), buffer.begin() + codePhase);
        codePhase = 0;
      }

      float codePhaseStep = codeFreq / sampleFreq;
      int blksize = std::ceil((codeLength-remCodePhase) / codePhaseStep);

      if (buffer.size() >= blksize) {

        // Generate Early CA Code.
        float tStartEarly = remCodePhase - earlyLateSpc;
        float tEndEarly = blksize*codePhaseStep+remCodePhase - earlyLateSpc;
        std::vector<float> earlyCode = linspace(tStartEarly, tEndEarly, blksize);
        std::for_each(earlyCode.begin(), earlyCode.end(), [this] (float x) {return caCode.at( std::ceil(x));});

        // Generate Late CA Code.
        float tStartLate = remCodePhase - earlyLateSpc;
        float tEndLate = blksize*codePhaseStep+remCodePhase + earlyLateSpc;
        std::vector<float> lateCode = linspace(tStartLate, tEndLate, blksize);
        std::for_each(lateCode.begin(), lateCode.end(), [this] (float x) {return caCode.at( std::ceil(x));});

        // Generate Late CA Code.
        float tStartPrompt = remCodePhase - earlyLateSpc;
        float tEndPrompt = blksize*codePhaseStep+remCodePhase;
        std::vector<float> promptCode = linspace(tStartPrompt, tEndPrompt, blksize);
        std::for_each(promptCode.begin(), promptCode.end(), [this] (float x) {return caCode.at( std::ceil(x));});

        // Figure out remaining code phase (uses tcode from Prompt CA Code generation):
        remCodePhase = linspace(tStartPrompt, tEndPrompt, blksize).back() + codePhaseStep - 1023;
        if(std::abs(remCodePhase) > codePhaseStep) {
          remCodePhase = copysign(1.0, remCodePhase) * codePhaseStep;
        } else remCodePhase = 0;

        std::vector<float> trgiArg = linspace(0, blksize * codePhaseStep, blksize);
        std::transform(trgiArg.begin(), trgiArg.end(), trgiArg.begin(), [this] (float x) { return (carrFreq * M_PI * 2 * x + remCarrPhase); });

        // Update remCarrPhase
        remCarrPhase = fmod(trgiArg.back(), 2 * M_PI);

        std::vector<float> qBaseBandSignal (trgiArg.size());
        std::vector<float> iBaseBandSignal (trgiArg.size());

        std::transform(trgiArg.begin(), trgiArg.end(), buffer.begin(), qBaseBandSignal.begin(), [this] (float x, float y) { return cos(x) * y; });
        std::transform(trgiArg.begin(), trgiArg.end(), buffer.begin(), iBaseBandSignal.begin(), [this] (float x, float y) { return sin(x) * y; });

        std::vector<float> corrVector(blksize);

        // declare variables for correlation results for early late and prompt codes with the signal (I_P is defined in the class)
        float I_E, Q_E, Q_P, I_L, Q_L;

        // do element wise multiplication and save SUM in the above variables

        std::transform(earlyCode.begin(), earlyCode.end(), qBaseBandSignal.begin(), corrVector.begin(), std::multiplies<float>());
        Q_E = std::accumulate(corrVector.begin(), corrVector.end(), 0.0);

        std::transform(earlyCode.begin(), earlyCode.end(), iBaseBandSignal.begin(), corrVector.begin(), std::multiplies<float>());
        I_E = std::accumulate(corrVector.begin(), corrVector.end(), 0.0);

        std::transform(promptCode.begin(), promptCode.end(), qBaseBandSignal.begin(), corrVector.begin(), std::multiplies<float>());
        Q_P = std::accumulate(corrVector.begin(), corrVector.end(), 0.0);

        std::transform(promptCode.begin(), promptCode.end(), iBaseBandSignal.begin(), corrVector.begin(), std::multiplies<float>());
        I_P = std::accumulate(corrVector.begin(), corrVector.end(), 0.0);

        std::transform(lateCode.begin(), lateCode.end(), qBaseBandSignal.begin(), corrVector.begin(), std::multiplies<float>());
        Q_L = std::accumulate(corrVector.begin(), corrVector.end(), 0.0);

        std::transform(lateCode.begin(), lateCode.end(), iBaseBandSignal.begin(), corrVector.begin(), std::multiplies<float>());
        I_L = std::accumulate(corrVector.begin(), corrVector.end(), 0.0);


        //  Find PLL error and update carrier NCO
        //  Implement carrier loop discriminator (phase detector)

        float carrError = atan(Q_P / I_P) / ( 2.0 * M_PI );

        // Implement carrier loop filter and generate NCO command

        float carrNco = oldCarrNco + tau1carr * (carrError - oldCarrError) + carrError * tau2carr;
        oldCarrNco   = carrNco;
        oldCarrError = carrError;


        //  Modify carrier freq based on NCO command
        carrFreq = carrFreqBasis + carrNco;

        float codeError = (sqrt(I_E * I_E + Q_E * Q_E) - sqrt(I_L * I_L + Q_L * Q_L)) / (sqrt(I_E * I_E + Q_E * Q_E) + sqrt(I_L * I_L + Q_L * Q_L));

        //  Implement code loop filter and generate NCO command
        float codeNco = oldCodeNco + tau1code * (codeError - oldCodeError) + codeError * tau2code;
        oldCodeNco   = codeNco;
        oldCodeError = codeError;
        

        //  Modify code freq based on NCO command
        codeFreq = chippingRate - codeNco;
        buffer.erase(buffer.begin(), buffer.begin() + blksize);
      }

      std::transform(out, out + noutput_items, out, [this](input_type x) {return I_P;});
      
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }



  } /* namespace gnss */
} /* namespace gr */

