/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "acquisition_impl.h"
#include <gnuradio/io_signature.h>
#include "generate_l1_ca.h"
#include "helper-functions.h"
#include <cmath>
#include <valarray>

#include <fstream>
#include <iterator>

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
      caCodesTable = makeComplexCaTable(samplesPerCode);
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
          std::complex<float> *dst = p1_fft.get_inbuf();
          memcpy(&dst[0], caCodesTable.at(PRN - 1).data(), sizeof(std::complex<float>) * samplesPerCode);
          p1_fft.execute();
          std::vector<std::complex<float>> caCodeFreqDom(samplesPerCode);
          memcpy(caCodeFreqDom.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

          // std::vector<int> caCode = generateCa(1, 0);
          // std::vector<std::complex<float>> caComplex;
          // for (auto i : caCode)
          // {
          //   caComplex.push_back(std::complex<float>(i * 1.0, 0.0));
          // }

          // gr::fft::fft_complex_fwd test_fft(1023, 1);
          // float a[]{1, -1, -1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1};
          // std::vector<std::complex<float>> b(65536);
          // for (int i = 0; i < 65536; i++)
          // {
          //   b.at(i) = caCodesTable.at(PRN - 1).at(i);
          // }
          // std::valarray<std::complex<float>> b(caCodesTable.at(PRN - 1).data(), 38192);
          // std::valarray<std::complex<float>> b(std::complex<float>(0, 0), 65536);
          // std::copy(&a[0], &a[0] + 38192, &b[27344]);

          // std::complex<float> *t = test_fft.get_inbuf();
          // memcpy(&t[0], caComplex.data(), sizeof(std::complex<float>) * 1023);
          // test_fft.execute();
          // std::vector<std::complex<float>> test(1023);
          // memcpy(test.data(), test_fft.get_outbuf(), sizeof(std::complex<float>) * 1023);

          // for (int ind = 0; ind < 20; ind++)
          // {
          //   if (PRN == 1)
          //     // std::cout << caCodesTable.at(PRN - 1).at(ind) << "    ";
          //     std::cout << test.at(ind).real() << "      " << test.at(ind).imag() << "i" << std::endl;
          // }
          // std::valarray<std::complex<double>> c(&b[0], 38192);
          // custom_fft(c);

          // for (int ind = 0; ind < 20; ind++)
          // {
          //   if (PRN == 1)
          //     std::cout << b[ind].real() << "      " << b[ind].imag() << "i" << std::endl;
          // }
          // // custom_fft(b);
          // for (int ind = 0; ind < 20; ind++)
          // {
          //   if (PRN == 1)
          //     std::cout << b[ind].real() << "      " << b[ind].imag() << "i" << std::endl;
          // }

          for (int frqBinIndex = 0; frqBinIndex < numberOfFrqBins; frqBinIndex++)
          {
            //  Generate carrier wave frequency grid (0.5kHz step)
            frqBins.at(frqBinIndex) = IF - 7000 + 0.5e3 * frqBinIndex;
            std::vector<std::complex<float>> IQSig1(samplesPerCode);
            std::vector<std::complex<float>> IQSig2(samplesPerCode);
            for (int k = 0; k < samplesPerCode; k++)
            {
              caCodeFreqDom.at(k) = std::conj(caCodeFreqDom.at(k));
              // Generate local sine and cosine
              double sinCarr = sin(k * 2 * M_PI * ts * frqBins.at(frqBinIndex));
              double cosCarr = cos(k * 2 * M_PI * ts * frqBins.at(frqBinIndex));
              float I1 = sinCarr * lognSignal.at(k);
              float Q1 = cosCarr * lognSignal.at(k);
              float I2 = sinCarr * lognSignal.at(k + samplesPerCode);
              float Q2 = cosCarr * lognSignal.at(k + samplesPerCode);

              // if (PRN == 1 && k == 71)
              // {
              //   std::cout << I1 << "    " << Q1 << std::endl;
              // }

              IQSig1.at(k) = std::complex<float>(I1, Q1);
              IQSig2.at(k) = std::complex<float>(I2, Q2);
            }
            // std::vector<float> test1(samplesPerCode), test2(samplesPerCode);
            // float max_test1{0}, max_test2{0};
            // int max_index1{0}, max_index2{0};
            // for (int i = 0; i < samplesPerCode; i++)
            // {

            //   float ac1 = std::abs(IQSig1.at(i));
            //   float ac2 = std::abs(IQSig2.at(i));
            //   test1.at(i) = ac1;
            //   test2.at(i) = ac2;

            //   if (max_test1 < ac1)
            //   {
            //     max_test1 = ac1;
            //     max_index1 = i;
            //   }
            //   if (max_test2 < ac2)
            //   {
            //     max_test2 = ac2;
            //     max_index2 = i;
            //   }
            // }

            // if (PRN == 1 && frqBinIndex == 0)
            // {
            //   std::ofstream output_file("./example.txt");
            //   std::ostream_iterator<float> output_iterator(output_file, "\n");
            //   std::copy(test1.begin(), test1.end(), output_iterator);
            // }

            if (PRN == 1)
            {
              // std::cout << "Max1:   " << std::distance(test1.begin(), std::max_element(test1.begin(), test1.end())) << "   Max2:   " << std::distance(test2.begin(), std::max_element(test2.begin(), test2.end())) << std::endl;
              // std::cout << IQSig2.at(71) << std::endl;

              // for (int i = 169; i < 174; i++)
              // {
              //   std::cout << std::abs(IQSig1.at(i)) << "   ";
              //   if (i == 173)
              //     std::cout << std::endl;
              // }

              //   for (int i = 38187; i < 38192; i++)
              //   {
              //     std::cout << std::abs(IQSig1.at(i)) << "   ";
              //     if (i == 38191)
              //       std::cout << std::endl;
            }

            // IQfreqDom1 = fft(I1 + j * Q1);
            // IQfreqDom2 = fft(I2 + j * Q2);
            std::complex<float> *ptr = p1_fft.get_inbuf();
            memcpy(&ptr[0], IQSig1.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft.execute();
            std::vector<std::complex<float>> IQfreqDom1(samplesPerCode);
            memcpy(IQfreqDom1.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            std::complex<float> *ptr2 = p1_fft.get_inbuf();
            memcpy(&ptr2[0], IQSig2.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft.execute();
            std::vector<std::complex<float>> IQfreqDom2(samplesPerCode);
            memcpy(IQfreqDom2.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            std::transform(IQfreqDom1.begin(), IQfreqDom1.end(),
                           caCodeFreqDom.begin(), IQfreqDom1.begin(),
                           std::multiplies<std::complex<float>>());
            std::transform(IQfreqDom2.begin(), IQfreqDom2.end(),
                           caCodeFreqDom.begin(), IQfreqDom2.begin(),
                           std::multiplies<std::complex<float>>());

            // TESTED UP UNTIL HERE ALL CORRECT

            std::vector<std::complex<float>> acqComplexRes1(samplesPerCode), acqComplexRes2(samplesPerCode);
            std::complex<float> *rev_ptr = p1_fft_rev.get_inbuf();
            memcpy(&rev_ptr[0], IQfreqDom1.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft_rev.execute();
            memcpy(acqComplexRes1.data(), p1_fft_rev.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            memcpy(&rev_ptr[0], IQfreqDom2.data(), sizeof(gr_complex) * samplesPerCode);
            p1_fft_rev.execute();
            memcpy(acqComplexRes2.data(), p1_fft_rev.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

            // std::vector<float> test1(samplesPerCode), test2(samplesPerCode);
            // float max_test1{0}, max_test2{0};
            // int max_index1{0}, max_index2{0};
            // for (int i = 0; i < samplesPerCode; i++)
            // {
            //   float ac1 = std::abs(acqComplexRes1.at(i));
            //   float ac2 = std::abs(acqComplexRes2.at(i));
            //   test1.at(i) = ac1;
            //   test2.at(i) = ac2;

            //   if (max_test1 < ac1)
            //   {
            //     max_test1 = ac1;
            //     max_index1 = i;
            //   }
            //   if (max_test2 < ac2)
            //   {
            //     max_test2 = ac2;
            //     max_index2 = i;
            //   }
            // }

            // if (PRN == 1)
            // {
            //   std::cout << max_index1 << ":   " << max_test1 << "        " << max_index2 << ":   " << max_test2 << std::endl;
            // }

            // if (PRN == 1 && frqBinIndex == 0)
            // {

            //   float a[]{2, -2, -3, 3, 3, 4, -1, -6, -7, 8, 3, -2, -2, 2, 2, -2};
            //   std::vector<std::complex<float>> testComplex(16);
            //   for (int i = 0; i < 16; i++)
            //   {
            //     testComplex.at(i) = std::complex<float>(a[i], i * 1.0);
            //   }
            //   gr::fft::fft_complex_fwd test_fft_fwd(16, 1);
            //   std::complex<float> *t_fwd = test_fft_fwd.get_inbuf();
            //   memcpy(&t_fwd[0], testComplex.data(), sizeof(std::complex<float>) * 16);
            //   test_fft_fwd.execute();
            //   std::vector<std::complex<float>> test_fwd(16);
            //   memcpy(test_fwd.data(), test_fft_fwd.get_outbuf(), sizeof(std::complex<float>) * 16);

            //   gr::fft::fft_complex_rev test_fft(16, 1);
            //   std::complex<float> *t = test_fft.get_inbuf();
            //   memcpy(&t[0], test_fwd.data(), sizeof(std::complex<float>) * 16);
            //   test_fft.execute();
            //   std::vector<std::complex<float>> test(16);
            //   memcpy(test.data(), test_fft.get_outbuf(), sizeof(std::complex<float>) * 16);

            //   for (int ind = 0; ind < 16; ind++)
            //   {
            //     std::cout << test.at(ind).real() / 16 << "      " << test.at(ind).imag() / 16 << "i" << std::endl;
            //   }
            // }

            std::vector<float> acqRes1, acqRes2;
            float max1{0}, max2{0};
            int index1{0}, index2{0};
            for (int i = 0; i < samplesPerCode; i++)
            {

              float ac1 = std::pow(std::abs(acqComplexRes1.at(i) / std::complex<float>(samplesPerCode, 0)), 2);
              float ac2 = std::pow(std::abs(acqComplexRes2.at(i) / std::complex<float>(samplesPerCode, 0)), 2);
              acqRes1.push_back(ac1);
              acqRes2.push_back(ac2);
            }
            // if (PRN == 1)
            // {

            //   std::cout << index1 << ":   " << max1 << "      " << index2 << ":   " << max2 << std::endl;
            // }
            max1 = *std::max_element(acqRes1.begin(), acqRes1.end());
            max2 = *std::max_element(acqRes2.begin(), acqRes2.end());
            index1 = std::distance(acqRes1.begin(), std::max_element(acqRes1.begin(), acqRes1.end()));
            index2 = std::distance(acqRes2.begin(), std::max_element(acqRes2.begin(), acqRes2.end()));

            if (max1 > max2)
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

          std::cout << "Peaksize 1: " << peakSize << "   SecondPeakSze: " << secondPeakSize << "    Ratio:  " << peakSize / secondPeakSize << "   PRN:  " << PRN << "    CodePhase:  " << codePhase << std::endl;

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
