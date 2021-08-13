/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "acquisition_impl.h"
#include <gnuradio/io_signature.h>
#include "generate_l1_ca.h"
#include <numeric>

namespace gr
{
  namespace gnss
  {

    using input_type = float;
    acquisition::sptr acquisition::make(float a_sampleFreq)
    {
      return gnuradio::make_block_sptr<acquisition_impl>(a_sampleFreq);
    }

    /*
 * The private constructor
 */
    acquisition_impl::acquisition_impl(float a_sampleFreq)
        : gr::sync_block("acquisition",
                         gr::io_signature::make(1 /* min inputs */,
                                                1 /* max inputs */,
                                                sizeof(input_type)),
                         gr::io_signature::make(0, 0, 0)),
          p1_fft_real(a_sampleFreq / 1000, 1), p1_fft(a_sampleFreq / 1000, 1), p1_fft_rev(a_sampleFreq / 1000, 1), sampleFreq{a_sampleFreq}
    {
      samplesPerCode = round(sampleFreq / (codeFreqBasis / codeLength));
      lognSignal.reserve(11 * samplesPerCode);
      caCodesTable = makeCaTable(samplesPerCode);
      frqBins.resize(numberOfFrqBins, 0);
      results.resize(numberOfFrqBins);
      ts = 1.0 / sampleFreq;
    }

    /*
 * Our virtual destructor.
 */
    acquisition_impl::~acquisition_impl()
    {
    }

    int acquisition_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
    {
      const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
      // Collect 11 ms of data required for acquisition cold start
      for (int i = 0; i < noutput_items; i++)
      {
        if (doColdStart && iterator < 11 * samplesPerCode)
        {
          lognSignal.push_back(in[i]);
          iterator++;
        }
        else
          break;
      }

      if (doColdStart && iterator == 11 * samplesPerCode)
      {
        for (int PRN = 1; PRN <= 32; PRN++)
        {
          float *dst = p1_fft_real.get_inbuf();
          memcpy(&dst[0], caCodesTable.at(PRN - 1).data(), sizeof(float) * samplesPerCode);
          p1_fft_real.execute();
          std::vector<std::complex<float>> caCodeFreqDom(samplesPerCode);
          memcpy(caCodeFreqDom.data(), p1_fft_real.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

          for (int ind = 0; ind < 100; ind++)
          {
            if (PRN == 1)
              std::cout << caCodeFreqDom.at(ind) << std::endl;
          }

          for (int frqBinIndex = 0; frqBinIndex < numberOfFrqBins; frqBinIndex++)
          {
            //  Generate carrier wave frequency grid (0.5kHz step)
            frqBins.at(frqBinIndex) = IF - 7000 + 0.5e3 * frqBinIndex;
            std::vector<std::complex<float>> IQSig1(samplesPerCode);
            std::vector<std::complex<float>> IQSig2(samplesPerCode);
            for (int k = 0; k < samplesPerCode; k++)
            {
              // caCodeFreqDom.at(i) = std::conj(caCodeFreqDom.at(i));
              // Generate local sine and cosine
              float cosCarr, sinCarr;
              sincosf((k * 2 * M_PI * ts * frqBins.at(frqBinIndex)), &cosCarr, &sinCarr);

              float I1 = sinCarr * lognSignal.at(k);
              float Q1 = cosCarr * lognSignal.at(k);
              float I2 = sinCarr * lognSignal.at(k + samplesPerCode);
              float Q2 = cosCarr * lognSignal.at(k + samplesPerCode);

              std::complex<float> IQCompl1(I1, Q1);
              std::complex<float> IQCompl2(I2, Q2);
              IQSig1.at(k) = IQCompl1;
              IQSig2.at(k) = IQCompl2;
            }

            // IQfreqDom1 = fft(I1 + j * Q1);
            // IQfreqDom2 = fft(I2 + j * Q2);
            std::complex<float> *dst = p1_fft.get_inbuf();
            memcpy(&dst[0], IQSig1.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft.execute();
            std::vector<std::complex<float>> IQfreqDom1(samplesPerCode);
            memcpy(IQfreqDom1.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            memcpy(&dst[0], IQSig2.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft.execute();
            std::vector<std::complex<float>> IQfreqDom2(samplesPerCode);
            memcpy(IQfreqDom2.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            std::transform(IQfreqDom1.begin(), IQfreqDom1.end(),
                           caCodeFreqDom.begin(), IQfreqDom1.begin(),
                           std::multiplies<std::complex<float>>());
            std::transform(IQfreqDom2.begin(), IQfreqDom2.end(),
                           caCodeFreqDom.begin(), IQfreqDom2.begin(),
                           std::multiplies<std::complex<float>>());

            memcpy(&dst[0], IQfreqDom1.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft_rev.execute();
            memcpy(IQfreqDom1.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            memcpy(&dst[0], IQfreqDom2.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft_rev.execute();
            memcpy(IQfreqDom2.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            // std::transform(IQfreqDom1.begin(), IQfreqDom1.end(), IQfreqDom1.begin(), [](std::complex<float> a)
            //                { return std::pow(std::abs(a), 2); });

            // std::transform(IQfreqDom2.begin(), IQfreqDom2.end(), IQfreqDom2.begin(), [](std::complex<float> a)
            //                { return std::pow(std::abs(a), 2); });
            std::vector<float> acqRes1, acqRes2;
            acqRes1.reserve(samplesPerCode);
            acqRes2.reserve(samplesPerCode);
            for (int i = 0; i < IQfreqDom1.size(); i++)
            {
              acqRes1.push_back(std::pow(std::abs(IQfreqDom1.at(i)), 2));
              acqRes2.push_back(std::pow(std::abs(IQfreqDom2.at(i)), 2));
            }

            // std::cout << "SUCCESS" << std::endl;
            if (*std::max_element(acqRes1.begin(), acqRes1.end()) > *std::max_element(acqRes2.begin(), acqRes2.end()))
              results.at(frqBinIndex) = acqRes1;
            else
              results.at(frqBinIndex) = acqRes2;
          }
          float peakSize{0};
          int frequencyBinIndex{0}, codePhase{0};

          for (int i = 0; i < results.size(); i++)
          {
            float max_element = *std::max_element(results.at(i).begin(), results.at(i).end());
            if (peakSize < max_element)
            {
              peakSize = max_element;
              frequencyBinIndex = i;
              codePhase = std::max_element(results.at(i).begin(), results.at(i).end()) - results.at(i).begin();
            }
          }
          int samplesPerCodeChip = std::round(sampleFreq / codeFreqBasis);
          int excludeRangeIndex1 = codePhase - samplesPerCodeChip;
          int excludeRangeIndex2 = codePhase + samplesPerCodeChip;
          int codePhaseRangeStart{0}, codePhaseRangeEnd{0};
          bool inclusive = true;
          if (excludeRangeIndex1 < 1)
          {
            codePhaseRangeStart = excludeRangeIndex2;
            codePhaseRangeEnd = samplesPerCode + excludeRangeIndex1;
          }
          else if (excludeRangeIndex2 >= samplesPerCode)
          {
            codePhaseRangeStart = excludeRangeIndex2 - samplesPerCode;
            codePhaseRangeEnd = excludeRangeIndex1;
          }
          else
          {
            codePhaseRangeStart = excludeRangeIndex1;
            codePhaseRangeEnd = excludeRangeIndex2;
            inclusive = false;
          }

          float secondPeakSize{0};

          for (int i = 0; i < samplesPerCode; i++)
          {
            if (inclusive && (i >= codePhaseRangeStart && i < codePhaseRangeEnd))
            {
              if (secondPeakSize < results.at(frequencyBinIndex).at(i))
                secondPeakSize = results.at(frequencyBinIndex).at(i);
            }
            else if (!inclusive && i < codePhaseRangeStart || i > codePhaseRangeEnd)
            {
              if (secondPeakSize < results.at(frequencyBinIndex).at(i))
                secondPeakSize = results.at(frequencyBinIndex).at(i);
            }
          }

          // if (peakSize / secondPeakSize > 2.5)
          // {
          //   std::vector<int> caCode = generateCa(PRN);
          //   // int codeValueIndex = floor((ts * (1 : 10 * samplesPerCode)) / (1 / codeFreqBasis));
          //   std::vector<float> xCarrier;
          //   xCarrier.reserve(samplesPerCode * 10);
          //   float longSignalMean = std::accumulate(lognSignal.begin(), lognSignal.end(), 0.0) / lognSignal.size();
          //   for (int i = 0; i < samplesPerCode * 10; i++)
          //   {
          //     int index = floor(ts * i * codeFreqBasis);
          //     int caCodeIndex = index % 1023;
          //     xCarrier.push_back((lognSignal.at(i + codePhase) - longSignalMean) * caCode.at(caCodeIndex));
          //   }

          //   int fftNumPts = 8 * pow(2, ceil(log2(xCarrier.size())));
          //   gr::fft::fft_real_fwd fft_num_pts(fftNumPts, 1);

          //   float *ptr = fft_num_pts.get_inbuf();
          //   memcpy(&ptr[0], xCarrier.data(), sizeof(float) * fftNumPts);
          //   fft_num_pts.execute();
          //   std::vector<std::complex<float>> fftxc(fftNumPts);
          //   memcpy(fftxc.data(), fft_num_pts.get_outbuf(), sizeof(gr_complex) * fftNumPts);
          //   std::vector<float> fftxcAbs;
          //   fftxcAbs.reserve(fftxc.size());
          //   for (auto val : fftxc)
          //   {
          //     fftxcAbs.push_back(std::abs(val));
          //   }

          //   int uniqFftPts = ceil((fftNumPts + 1) / 2);

          //   int fftMaxIndex = std::max_element(fftxcAbs.begin() + 4, fftxcAbs.end() - 5) - fftxcAbs.begin() - 4;
          //   float fftMax = *std::max_element(fftxcAbs.begin() + 4, fftxcAbs.end() - 5);

          //   float carrFreq = fftMaxIndex * sampleFreq / fftNumPts;
          //   // codePhase

          //   std::cout << "PRN: " << PRN << " -> CarrFreq: " << carrFreq << ", CodePhase: " << codePhase << std::endl;
          // }
        }
        doColdStart = false;
        iterator = 0;
      }

      // Do <+signal processing+>

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace gnss */
} /* namespace gr */
