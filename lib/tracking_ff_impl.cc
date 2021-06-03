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

        // declare variables for correlation results for early late and prompt codes with the signal (I_P is defined in the class)
        float I_E {0}, Q_E {0}, Q_P {0}, I_P {0}, I_L {0}, Q_L {0};

        // do element wise multiplication and save SUM in the above variables

        float tStartEarly = remCodePhase - earlyLateSpc;
        float tEndEarly = blksize*codePhaseStep+remCodePhase - earlyLateSpc;
        float earlyCode {0};

        float tStartLate = remCodePhase + earlyLateSpc;
        float tEndLate = blksize*codePhaseStep+remCodePhase + earlyLateSpc;
        float lateCode {0};

        float tStartPrompt = remCodePhase;
        float tEndPrompt = blksize*codePhaseStep+remCodePhase;
        float promptCode {0};

        float codeStep = tEndPrompt / tStartPrompt;

        for(int i = 0; i< blksize; i++) {

            // Generate Early CA Code.
            earlyCode = caCode.at( std::ceil( tStartEarly + codeStep * i ) );
            
            // Generate Late CA Code.
            lateCode = caCode.at( std::ceil( tStartLate + codeStep * i ) );

            // Generate Prompt CA Code.
            promptCode = caCode.at( std::ceil( tStartPrompt + codeStep * i ) );



            // Figure out remaining code phase (uses tcode from Prompt CA Code generation):

            float trigArg = (carrFreq * 2 * M_PI * (i / sampleFreq)) + remCarrPhase;
            
            // Update remCarrPhase
            if(i == blksize -1) remCarrPhase = remainderf(trigArg, 2 * M_PI);
            
            float qSignal = cos(trigArg) * buffer.at(i);
            float iSignal = sin(trigArg) * buffer.at(i);

            if ( output == 0 &&( i == 0 || i == std::abs(blksize / 2) || i == blksize -1 ) ) {
              std::cout<<"Early code: "<<earlyCode<<std::endl;
              std::cout<<"Late code: "<<lateCode<<std::endl;
              std::cout<<"Prompt code: "<<promptCode<<std::endl;
              std::cout<<"iSignal: "<<iSignal<<std::endl;
            }

            Q_E += earlyCode * qSignal;
            I_E += earlyCode * iSignal;
            Q_P += promptCode * qSignal;
            I_P += promptCode * iSignal;
            Q_L += lateCode * qSignal;
            I_L += lateCode * iSignal;
        }

        
        remCodePhase = tEndPrompt - codeStep + codePhaseStep - 1023;
        if(std::abs(remCodePhase) > codePhaseStep) {
          remCodePhase = copysign(1.0, remCodePhase) * codePhaseStep;
        } else remCodePhase = 0;
            

        // Update output value to I_P
        output += 1;   

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

      for(int i = 0; i< noutput_items; i++) {
        out[i] = output;
      }
      // Tell runtime system how many output items we produced.
      return noutput_items;
    }



  } /* namespace gnss */
} /* namespace gr */

