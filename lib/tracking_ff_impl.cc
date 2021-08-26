/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "generate_l1_ca.h"
#include "helper-functions.h"
#include "tracking_ff_impl.h"
#include "acqResults.h"
#include <gnuradio/io_signature.h>
#include <algorithm> // std::transform std::for_each
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <iostream>

namespace gr
{
    namespace gnss
    {

        using input_type = float;
        using output_type = float;
        tracking_ff::sptr tracking_ff::make(int _channelNum, float _sampleFreq)
        {
            return gnuradio::make_block_sptr<tracking_ff_impl>(_channelNum, _sampleFreq);
        }

        /*
 * The private constructor
 */

        tracking_ff_impl::tracking_ff_impl(int _channelNum, float _sampleFreq)
            : gr::sync_block("tracking_ff",
                             gr::io_signature::make(
                                 1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                             gr::io_signature::make(
                                 1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
              channelNum{_channelNum}, samplePeriod{1 / _sampleFreq}
        {
            // channel = new Channel(21, 9547426.34201050, 13404, 'T');
            message_port_register_in(pmt::string_to_symbol("acquisition"));
            set_msg_handler(pmt::mp("acquisition"), [this](const pmt::pmt_t &msg)
                            {
                                auto msg_key = pmt::car(msg);
                                auto mag_val = pmt::cdr(msg);
                                if (pmt::symbol_to_string(msg_key) == "acq_start")
                                {
                                    auto recChannelNumber = pmt::to_long(mag_val);
                                    if (recChannelNumber == channelNum)
                                    {
                                        // Reset count of totalSamples to correctly adjust codePhase received from acquisition
                                        // std::cout << "TRACKING == " << test << std::endl;
                                        totalSamples = 0;
                                    }
                                }
                                else if (pmt::symbol_to_string(msg_key) == "acq_result")
                                {
                                    AcqResults acqResult = *(reinterpret_cast<const AcqResults *>(pmt::blob_data(mag_val)));
                                    if (acqResult.channelNumber == channelNum)
                                    {
                                        // std::cout << "TRACKING  total samples:  " << totalSamples << std::endl;

                                        codeFreq = 1023000;
                                        codePhaseStep = codeFreq * samplePeriod;
                                        blksize = ceil(codeLength / codePhaseStep);

                                        float totalSamplesFromStart = totalSamples - acqResult.codePhase;
                                        float fullMsPassed = ceil(totalSamplesFromStart / blksize);
                                        codePhase = fullMsPassed * blksize - totalSamplesFromStart;
                                        doTracking = false;
                                        carrFreq = acqResult.carrFreq;
                                        carrFreqBasis = acqResult.carrFreq;
                                        PRN = acqResult.PRN;
                                        // std::cout << "PRN: " << PRN << " -> CarrFreq: " << carrFreq << ", CodePhase: " << codePhase << std::endl;
                                    }
                                }
                            });

            paddedCaTable.reserve(33);
            makePaddedCaTable(paddedCaTable);
            codePhaseStep = codeFreq * samplePeriod;
            blksize = ceil((codeLength - remCodePhase) / codePhaseStep);
            calcloopCoef(
                tau1carr, tau2carr, pllNoiseBandwidth, pllDampingRatio, loopGainCarr, PDI);
            calcloopCoef(
                tau1code, tau2code, dllNoiseBandwidth, dllDampingRatio, loopGainCode, PDI);
        }

        /*
 * Our virtual destructor.
 */
        tracking_ff_impl::~tracking_ff_impl() {}
        void tracking_ff_impl::reset()
        {
            remCodePhase = 0.0;
            remCarrPhase = 0.0;
            oldCarrNco = 0.0;
            oldCarrError = 0.0;
            oldCodeNco = 0.0;
            oldCodeError = 0.0;
            carrFreq = 0.0;
            codeFreq = 1023000;
            iterator = 0;
            Q_E = I_E = Q_P = I_P = Q_L = I_L = 0.0;
        }

        int tracking_ff_impl::work(int noutput_items,
                                   gr_vector_const_void_star &input_items,
                                   gr_vector_void_star &output_items)
        {
            const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
            output_type *out = reinterpret_cast<output_type *>(output_items[0]);

            // Declare Early Late and Prompt code variables and their starting points
            float tStartEarly = remCodePhase - earlyLateSpc;
            float tStartLate = remCodePhase + earlyLateSpc;
            float tStartPrompt = remCodePhase;
            float tEndPrompt = blksize * codePhaseStep + remCodePhase;
            for (int i = 0; i < noutput_items; i++)
            {
                test = i;

                totalSamples++;

                if (true)
                {
                    if (!doTracking && PRN != 0)
                    {
                        if (codePhase > 0)
                            codePhase--;
                        if (codePhase == 0)
                        {
                            reset();
                            doTracking = true;
                        }
                    }

                    if (doTracking)
                    {
                        float iteratorStep = codePhaseStep * iterator;

                        // Generate Early CA Code.
                        int earlyCode{}, lateCode{}, promptCode{};

                        earlyCode = paddedCaTable.at(PRN).at(std::ceil(tStartEarly + iteratorStep));

                        // Generate Late CA Code.
                        lateCode = paddedCaTable.at(PRN).at(std::ceil(tStartLate + iteratorStep));

                        // Generate Prompt CA Code.
                        promptCode = paddedCaTable.at(PRN).at(std::ceil(tStartPrompt + iteratorStep));

                        float trigArg =
                            (carrFreq * 2 * M_PI * (iterator * samplePeriod)) + remCarrPhase;

                        float carrSin;
                        float carrCos;
                        sincosf(trigArg, &carrSin, &carrCos);
                        float qSignal = in[i] * carrCos;
                        float iSignal = in[i] * carrSin;

                        Q_E += earlyCode * qSignal;
                        I_E += earlyCode * iSignal;
                        Q_P += promptCode * qSignal;
                        I_P += promptCode * iSignal;
                        Q_L += lateCode * qSignal;
                        I_L += lateCode * iSignal;

                        // When 1 ms of data is processed update code and carr remaining phase values
                        // and reset iterator else incr iterator
                        if (iterator == blksize - 1)
                        {

                            // Update output value to I_P
                            output = I_P;
                            remCarrPhase =
                                fmodf((carrFreq * 2 * M_PI * ((blksize)*samplePeriod) + remCarrPhase),
                                      (2 * M_PI));

                            // Update remaining Code Phase once per ms
                            remCodePhase = tEndPrompt - 1023.0;
                            //  Find PLL error and update carrier NCO
                            //  Implement carrier loop discriminator (phase detector)
                            float carrError = atan(Q_P / I_P) / (2.0 * M_PI);

                            // Implement carrier loop filter and generate NCO command

                            float carrNco = oldCarrNco + tau1carr * (carrError - oldCarrError) +
                                            carrError * tau2carr;
                            oldCarrNco = carrNco;
                            oldCarrError = carrError;
                            //  Modify carrier freq based on NCO command
                            carrFreq = carrFreqBasis + carrNco;

                            float sqrtEarly = sqrt(I_E * I_E + Q_E * Q_E);
                            float sqrtLate = sqrt(I_L * I_L + Q_L * Q_L);
                            float codeError = (sqrtEarly - sqrtLate) / (sqrtEarly + sqrtLate);

                            //  Implement code loop filter and generate NCO command
                            float codeNco = oldCodeNco + tau1code * (codeError - oldCodeError) +
                                            codeError * tau2code;
                            oldCodeNco = codeNco;
                            oldCodeError = codeError;

                            //  Modify code freq based on NCO command and update codePhaseStep
                            codeFreq = chippingRate - codeNco;
                            codePhaseStep = codeFreq * samplePeriod;

                            // Reset early late and prompt correlation results and set iterator to 0
                            Q_E = I_E = Q_P = I_P = Q_L = I_L = 0;
                            iterator = 0;
                            // update blksize
                            blksize = std::ceil((codeLength - remCodePhase) / codePhaseStep);

                            ms++;
                        }
                        else
                        {
                            iterator++;
                        }
                    }
                    out[i] = output;
                }
            }

            // Tell runtime system how many output items we produced.
            return noutput_items;
        }

    } /* namespace gnss */
} /* namespace gr */
