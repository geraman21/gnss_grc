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
#include <iostream>


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
      codePhaseStep = codeFreq / sampleFreq;
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
      // buffer.insert(buffer.end(), in, in + noutput_items);
      // std::cout<<"Size of in: "<<sizeof(input_items[0])/sizeof(in[0])<<std::endl;
      // std::cout<<"nuoutput_items: "<<noutput_items<<std::endl;
      // std::cout<<"Buffer Size: "<<buffer.size()<<std::endl;

      // Align the phase of the signal
      if (codePhase > 0) {
        // buffer.erase(buffer.begin(), buffer.begin() + codePhase);
        codePhase -= noutput_items;
        std::cout<<codePhase<<std::endl;
        std::cout<<noutput_items<<std::endl;
      }

      // Declare Early Late and Prompt code variables and their starting points
      float tStartEarly = remCodePhase - earlyLateSpc;
      float tStartLate = remCodePhase + earlyLateSpc;
      float tStartPrompt = remCodePhase;
      float tEndPrompt = blksize*codePhaseStep+remCodePhase;

      for(int i = 0; i < noutput_items; i++) {
        if ( codePhase < 0 && i == (noutput_items + codePhase + 1)) {
          std::cout<<I_P<<std::endl;
          codePhase = 0;}
          
        if (codePhase == 0)    { 
          // Generate Early CA Code.
          float earlyCode = caCode.at( std::ceil( tStartEarly + codePhaseStep * iterator ) );
          
          // Generate Late CA Code.
          float lateCode = caCode.at( std::ceil( tStartLate + codePhaseStep * iterator ) );

          // Generate Prompt CA Code.
          float promptCode = caCode.at( std::ceil( tStartPrompt + codePhaseStep * iterator ) );

          float trigArg = (carrFreq * 2 * M_PI * (iterator / sampleFreq)) + remCarrPhase;
          
          float qSignal = cos(trigArg) * in[i];
          float iSignal = sin(trigArg) * in[i];

          Q_E += earlyCode * qSignal;
          I_E += earlyCode * iSignal;
          Q_P += promptCode * qSignal;
          I_P += promptCode * iSignal;
          Q_L += lateCode * qSignal;
          I_L += lateCode * iSignal;
        
          // When 1 ms of data is processed update code and carr remaining phase values and reset iterator else incr iterator
          if(iterator == blksize -1) {
            
            // Update output value to I_P
            output = I_P;   

            remCarrPhase = remainderf(trigArg, 2 * M_PI);
            
            // Update remaining Code Phase once per ms
            remCodePhase = tEndPrompt - 1023;
            if(std::abs(remCodePhase) > codePhaseStep) {
              remCodePhase = copysign(1.0, remCodePhase) * codePhaseStep;
            } else remCodePhase = 0;

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
            

            //  Modify code freq based on NCO command and update codePhaseStep
            codeFreq = chippingRate - codeNco;
            codePhaseStep = codeFreq / sampleFreq;

            // Reset early late and prompt correlation results and set iterator to 0
            Q_E = I_E = Q_P = I_P = Q_L = I_L = 0;
            iterator = 0;
            // udate blksize
            blksize = std::ceil((codeLength-remCodePhase) / codePhaseStep);
          }  else  iterator++;
        }
        out[i] = output;
      }
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }



  } /* namespace gnss */
} /* namespace gr */

