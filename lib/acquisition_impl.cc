/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "acquisition_impl.h"
#include "generate_l1_ca.h"
#include "helper-functions.h"
#include <cmath>
#include <gnuradio/io_signature.h>
#include <numeric>

namespace gr {
namespace gnss {

using input_type = float;
acquisition::sptr acquisition::make(float a_sampleFreq, float im_freq, int a_channelNum) {
  return gnuradio::make_block_sptr<acquisition_impl>(a_sampleFreq, im_freq, a_channelNum);
}

/*
 * The private constructor
 */
acquisition_impl::acquisition_impl(float a_sampleFreq, float im_freq, int a_channelNum)
    : gr::sync_block(
          "acquisition",
          gr::io_signature::make(0 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(0, 0, 0)),
      sampleFreq{a_sampleFreq}, channelNum{a_channelNum}, IF{im_freq} {
  message_port_register_in(pmt::string_to_symbol("data_vector"));
  message_port_register_out(pmt::string_to_symbol("acquisition"));
  samplesPerCode = round(sampleFreq / (codeFreqBasis / codeLength));
  longSignal.reserve(2 * samplesPerCode);
  caCodesTable = makeComplexCaTable(samplesPerCode);
  ts = 1.0 / sampleFreq;
  channels.resize(channelNum, 0);
  set_msg_handler(pmt::mp("data_vector"), [this](const pmt::pmt_t &msg) {
    auto msg_key = pmt::car(msg);
    auto msg_val = pmt::cdr(msg);
    int receivedPRN = pmt::to_long(msg_key);
    const gr_complex *data = reinterpret_cast<const gr_complex *>(pmt::blob_data(msg_val));
    longSignal.assign(data, data + longSignal.capacity());
    acqResults.clear();
    std::cout << "Acquisition Cold Start Initiated" << std::endl;
    std::cout << "(  ";
    for (int PRN = 1; PRN <= 32; PRN++) {
      if (std::find(channels.begin(), channels.end(), PRN) != channels.end()) {
        continue;
      }
      AcqResults result =
          checkIfChannelPresent(PRN, ts, IF, complexCaTable.at(PRN - 1), longSignal);
      std::cout.precision(1);
      result.PRN ? std::cout << std::fixed << PRN << " [ " << result.peakMetric << " ]   "
                 : std::cout << "   .   ";
      if (result.PRN)
        acqResults.push_back(result);
    }
    std::cout << ")" << std::endl;
    bool newChannelAcquired = false;
    if (acqResults.size() > 0) {
      std::sort(acqResults.begin(), acqResults.end(),
                [](AcqResults a, AcqResults b) { return (a.peakMetric < b.peakMetric); });
      // Send active channels to respective tracking blocks if no specific PRN provided
      for (int i = 0; i < channelNum; i++) {
        if (channels.at(i) == receivedPRN) {
          if (acqResults.size() == 0) {
            newChannelAcquired = false;
            break;
          }

          acqResults.back().channelNumber = i;
          channels.at(i) = acqResults.back().PRN;

          auto size = sizeof(AcqResults);
          auto pmt = pmt::make_blob(reinterpret_cast<void *>(&acqResults.back()), size);
          message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_result"), pmt));
          newChannelAcquired = true;
          std::cout << "Assigned channel   " << acqResults.back().channelNumber << "("
                    << receivedPRN << ")"
                    << "    new PRN value   " << acqResults.back().PRN << std::endl;
          acqResults.pop_back();
        }
      }
    }
    if (!newChannelAcquired) {
      auto size = sizeof(AcqResults);
      AcqResults emptyResult = AcqResults();
      emptyResult.PRN = receivedPRN;
      auto pmt = pmt::make_blob(reinterpret_cast<void *>(&emptyResult), size);
      message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_result"), pmt));
      message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_restart"), pmt));
    }
  });

  complexCaTable = makeComplexCaTable(samplesPerCode);
}

/*
 * Our virtual destructor.
 */
acquisition_impl::~acquisition_impl() {}

int acquisition_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
