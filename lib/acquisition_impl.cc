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
acquisition::sptr acquisition::make(float a_sampleFreq, int a_channelNum) {
  return gnuradio::make_block_sptr<acquisition_impl>(a_sampleFreq, a_channelNum);
}

/*
 * The private constructor
 */
acquisition_impl::acquisition_impl(float a_sampleFreq, int a_channelNum)
    : gr::sync_block(
          "acquisition",
          gr::io_signature::make(0 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(0, 0, 0)),
      sampleFreq{a_sampleFreq}, channelNum{a_channelNum} {
  message_port_register_in(pmt::string_to_symbol("data_vector"));
  message_port_register_out(pmt::string_to_symbol("acquisition"));
  samplesPerCode = round(sampleFreq / (codeFreqBasis / codeLength));
  longSignal.reserve(11 * samplesPerCode);
  caCodesTable = makeComplexCaTable(samplesPerCode);
  ts = 1.0 / sampleFreq;

  set_msg_handler(pmt::mp("data_vector"), [this](const pmt::pmt_t &msg) {
    const float *data = reinterpret_cast<const float *>(pmt::blob_data(msg));
    longSignal.assign(data, data + longSignal.capacity());
    // if (doColdStart) {
    std::cout << "Acquisition Cold Start Initiated" << std::endl;
    std::cout << "(  ";
    for (int PRN = 1; PRN <= 32; PRN++) {
      AcqResults result = performAcquisition(PRN, ts, complexCaTable.at(PRN - 1), longSignal);
      result.PRN ? std::cout << result.PRN << "   " : std::cout << ".   ";
      if (result.PRN != 0)
        acqResults.push_back(result);
    }
    std::cout << ")" << std::endl;
    std::sort(acqResults.begin(), acqResults.end(),
              [](AcqResults a, AcqResults b) { return (a.peakMetric > b.peakMetric); });
    acqResults.resize(channelNum);
    doColdStart = false;
    // Start collecting acq data for the first channel
    for (int i = 0; i < acqResults.size(); i++) {
      acqResults.at(i).channelNumber = i;
      auto size = sizeof(AcqResults);
      auto pmt = pmt::make_blob(reinterpret_cast<void *>(&acqResults.at(i)), size);
      message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_result"), pmt));
    }
    // }
    // else {
    //   if (acqResults.size() > 0) {
    //     // std::cout << "Acquiring for PRN:  " << acqResults.front().PRN << "
    //     // channel strength:   " << acqResults.front().peakMetric << std::endl;
    //     AcqResults channelAcqResult = performAcquisition(
    //         acqResults.front().PRN, ts, complexCaTable.at(acqResults.front().PRN - 1),
    //         longSignal);
    //     channelAcqResult.channelNumber = channelNum - acqResults.size();
    //     if (channelAcqResult.PRN != 0) {
    //       auto size = sizeof(AcqResults);
    //       auto pmt = pmt::make_blob(reinterpret_cast<void *>(&channelAcqResult), size);
    //       message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_result"), pmt));
    //       // std::cout << "Acquisition Result Delivered:   Channel Number:   "
    //       //           << channelAcqResult.channelNumber << "    PRN: " <<
    //       //           channelAcqResult.PRN << "   CarrFreq: " <<
    //       //           channelAcqResult.carrFreq << ", CodePhase: " <<
    //       //           channelAcqResult.codePhase << std::endl;
    //     }
    //     acqResults.erase(acqResults.begin());
    //     // Notify next channel that Acquisition data for it is now being
    //     // collected
    //     if (acqResults.size() > 0)
    //       message_port_pub(
    //           pmt::mp("acquisition"),
    //           pmt::cons(pmt::mp("acq_start"), pmt::from_long(channelAcqResult.channelNumber +
    //           1)));
    //     else {
    //       doColdStart = true;
    //       std::cout << "Acquisition is finished" << std::endl;
    //     }
    //   } else {
    //     doColdStart = true;
    //     std::cout << "Acquisition is finished" << std::endl;
    //   }
    // }
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
