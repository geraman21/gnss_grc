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
#include <numeric>

namespace gr
{
  namespace gnss
  {

    using input_type = float;
    acquisition::sptr acquisition::make(float a_sampleFreq, int a_channelNum)
    {
      return gnuradio::make_block_sptr<acquisition_impl>(a_sampleFreq, a_channelNum);
    }

    /*
 * The private constructor
 */
    acquisition_impl::acquisition_impl(float a_sampleFreq, int a_channelNum)
        : gr::sync_block("acquisition",
                         gr::io_signature::make(0 /* min inputs */,
                                                1 /* max inputs */,
                                                sizeof(input_type)),
                         gr::io_signature::make(0, 0, 0)),
          p1_fft_real(a_sampleFreq / 1000, 1), p1_fft(a_sampleFreq / 1000, 1), p1_fft_rev(a_sampleFreq / 1000, 1), sampleFreq{a_sampleFreq}, channelNum{a_channelNum}
    {
      message_port_register_in(pmt::string_to_symbol("data_vector"));
      message_port_register_out(pmt::string_to_symbol("acquisition"));
      samplesPerCode = round(sampleFreq / (codeFreqBasis / codeLength));
      caCodesTable = makeComplexCaTable(samplesPerCode);
      frqBins.resize(numberOfFrqBins, 0);
      results.resize(numberOfFrqBins);
      ts = 1.0 / sampleFreq;

      set_msg_handler(pmt::mp("data_vector"), [this](const pmt::pmt_t &msg)
                      {
                        longSignal = *(reinterpret_cast<const std::vector<float> *>(pmt::blob_data(msg)));
                        if (doColdStart)
                        {
                          std::cout << "Acquisition Cold Start Initiated" << std::endl;
                          std::cout << "(  ";
                          for (int PRN = 1; PRN <= 32; PRN++)
                          {
                            AcqResults result = performAcquisition(PRN);
                            result.PRN ? std::cout << result.PRN << "   " : std::cout << ".   ";
                            if (result.PRN != 0)
                              acqResults.push_back(result);
                          }
                          std::cout << ")" << std::endl;
                          std::sort(acqResults.begin(), acqResults.end(), [](AcqResults a, AcqResults b)
                                    { return (a.peakMetric > b.peakMetric); });
                          acqResults.resize(channelNum);
                          doColdStart = false;
                          // Notify first channel that Acquisition data for it is now being collected
                          message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_start"), pmt::from_long(0)));
                        }
                        else
                        {
                          if (acqResults.size() > 0)
                          {
                            std::cout << "Acquiring for PRN:  " << acqResults.front().PRN << std::endl;
                            AcqResults channelAcqResult = performAcquisition(acqResults.front().PRN);
                            if (channelAcqResult.PRN != 0)
                            {
                              channelAcqResult.channelNumber = channelNum - acqResults.size();
                              acqResults.erase(acqResults.begin());
                              std::cout << "Acquisition Result Delivered:   Channel Number:   "
                                        << channelAcqResult.channelNumber << "    PRN: " << channelAcqResult.PRN << "   CarrFreq: " << channelAcqResult.carrFreq << ", CodePhase: " << channelAcqResult.codePhase << std::endl;
                              auto size = sizeof(AcqResults);
                              auto pmt = pmt::make_blob(reinterpret_cast<void *>(&channelAcqResult), size);
                              message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_result"), pmt));

                              // Notify next channel that Acquisition data for it is now being collected
                              message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_start"), pmt::from_long(channelAcqResult.channelNumber + 1)));
                            }
                          }
                          else
                          {
                            doColdStart = true;
                            std::cout << "Acquisition is finished" << std::endl;
                          }
                        }
                      });
    }

    /*
 * Our virtual destructor.
 */
    acquisition_impl::~acquisition_impl()
    {
    }

    AcqResults acquisition_impl::performAcquisition(int PRN)
    {
      std::complex<float> *dst = p1_fft.get_inbuf();
      memcpy(&dst[0], caCodesTable.at(PRN - 1).data(), sizeof(std::complex<float>) * samplesPerCode);
      p1_fft.execute();
      std::vector<std::complex<float>> caCodeFreqDom(samplesPerCode);
      memcpy(caCodeFreqDom.data(), p1_fft.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

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
          float I1 = sinCarr * longSignal.at(k);
          float Q1 = cosCarr * longSignal.at(k);
          float I2 = sinCarr * longSignal.at(k + samplesPerCode);
          float Q2 = cosCarr * longSignal.at(k + samplesPerCode);

          IQSig1.at(k) = std::complex<float>(I1, Q1);
          IQSig2.at(k) = std::complex<float>(I2, Q2);
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

        std::vector<std::complex<float>> acqComplexRes1(samplesPerCode), acqComplexRes2(samplesPerCode);
        std::complex<float> *rev_ptr = p1_fft_rev.get_inbuf();
        memcpy(&rev_ptr[0], IQfreqDom1.data(), sizeof(gr_complex) * samplesPerCode);
        p1_fft_rev.execute();
        memcpy(acqComplexRes1.data(), p1_fft_rev.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

        memcpy(&rev_ptr[0], IQfreqDom2.data(), sizeof(gr_complex) * samplesPerCode);
        p1_fft_rev.execute();
        memcpy(acqComplexRes2.data(), p1_fft_rev.get_outbuf(), sizeof(gr_complex) * samplesPerCode);

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

      if (peakSize / secondPeakSize > 2.5)
      {
        std::vector<int> caCode = generateCa(PRN);
        std::vector<std::complex<float>> xCarrier;
        xCarrier.reserve(samplesPerCode * 10);
        float longSignalMean = std::accumulate(longSignal.begin(), longSignal.end(), 0.0) / longSignal.size();
        for (int i = 0; i < samplesPerCode * 10; i++)
        {
          int index = floor(ts * i * codeFreqBasis);
          int caCodeIndex = index % 1023;
          xCarrier.push_back(std::complex<float>((longSignal.at(i + codePhase) - longSignalMean) * caCode.at(caCodeIndex), 0.0));
        }

        int fftNumPts = 8 * pow(2, ceil(log2(xCarrier.size())));
        gr::fft::fft_complex_fwd fft_num_pts(fftNumPts, 1);

        std::complex<float> *ptr = fft_num_pts.get_inbuf();
        memcpy(&ptr[0], xCarrier.data(), sizeof(gr_complex) * samplesPerCode * 10);
        fft_num_pts.execute();
        std::vector<std::complex<float>> fftxc(fftNumPts);
        memcpy(fftxc.data(), fft_num_pts.get_outbuf(), sizeof(gr_complex) * fftNumPts);
        std::vector<float> fftxcAbs;
        fftxcAbs.reserve(fftxc.size());
        for (auto val : fftxc)
        {
          fftxcAbs.push_back(std::abs(val));
        }

        int uniqFftPts = ceil((fftNumPts + 1) / 2);

        int fftMaxIndex = std::max_element(fftxcAbs.begin() + 4, fftxcAbs.begin() + uniqFftPts - 5) - fftxcAbs.begin() - 4;
        float carrFreq = fftMaxIndex * sampleFreq / fftNumPts;

        // std::cout << "PRN: " << PRN << " -> CarrFreq: " << carrFreq << ", CodePhase: " << codePhase << std::endl;
        AcqResults result(PRN, carrFreq, codePhase, peakSize / secondPeakSize);
        return result;

        // std::sort(acqResults.begin(), acqResults.end(), [](AcqResults a, AcqResults b)
        //           { return (a.peakMetric > b.peakMetric); });

        // auto size = sizeof(AcqResults) * acqResults.size();
        // auto pmt = pmt::make_blob(reinterpret_cast<void *>(&acqResults), size);
        // message_port_pub(pmt::string_to_symbol("acquisition"), pmt);
      }
      else
        return AcqResults();
    }

    int acquisition_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
    {
      const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
      return noutput_items;
    }

  } /* namespace gnss */
} /* namespace gr */
