/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "channel_starter_impl.h"
#include "acqResults.h"
#include "generate_l1_ca.h"
#include "helper-functions.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {

using input_type = float;
channel_starter::sptr channel_starter::make(int attempts, float s_sampleFreq) {
  return gnuradio::make_block_sptr<channel_starter_impl>(attempts, s_sampleFreq);
}

/*
 * The private constructor
 */
channel_starter_impl::channel_starter_impl(int attempts, float s_sampleFreq)
    : gr::sync_block(
          "channel_starter",
          gr::io_signature::make(0 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(0, 0, 0)),
      attemptsNum{attempts}, sampleFreq{s_sampleFreq} {
  message_port_register_in(pmt::string_to_symbol("data_vector"));
  message_port_register_out(pmt::string_to_symbol("acquisition"));
  samplesPerCode = round(sampleFreq / (codeFreqBasis / codeLength));
  longSignal.reserve(11 * samplesPerCode);
  ts = 1.0 / sampleFreq;

  set_msg_handler(pmt::mp("data_vector"), [this](const pmt::pmt_t &msg) {
    auto msg_key = pmt::car(msg);
    auto msg_val = pmt::cdr(msg);
    int receivedPRN = pmt::to_long(msg_key);
    std::cout << "Starter  working  PRN:   " << receivedPRN << std::endl;
    if (PRN != receivedPRN) {
      attemptsLeft = attemptsNum;
      PRN = receivedPRN;
      complexCaVector = makeComplexCaVector(samplesPerCode, PRN);
    }
    if (attemptsLeft > 0) {
      const float *data = reinterpret_cast<const float *>(pmt::blob_data(msg_val));
      longSignal.assign(data, data + longSignal.capacity());
      AcqResults acqResult = performAcquisition(PRN, ts, complexCaVector, longSignal);
      auto size = sizeof(AcqResults);
      auto pmt = pmt::make_blob(reinterpret_cast<void *>(&acqResult), size);
      message_port_pub(pmt::mp("acquisition"), pmt::cons(pmt::mp("acq_start"), pmt));
      attemptsLeft--;
    }
  });
}

/*
 * Our virtual destructor.
 */
channel_starter_impl::~channel_starter_impl() {}

int channel_starter_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */