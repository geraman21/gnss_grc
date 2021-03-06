/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "data_distributor_impl.h"
#include "acqResults.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {
using input_type = gr_complex;
using output_type = gr_complex;
data_distributor::sptr data_distributor::make(float numSamples) {
  return gnuradio::make_block_sptr<data_distributor_impl>(numSamples);
}

/*
 * The private constructor
 */
data_distributor_impl::data_distributor_impl(float numSamples)
    : gr::sync_block(
          "data_distributor",
          gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
      samplesToSend{(unsigned int)numSamples} {
  lognSignal.reserve(samplesToSend);
  message_port_register_out(pmt::string_to_symbol("data_vector"));
  message_port_register_in(pmt::string_to_symbol("acquisition"));
  set_msg_handler(pmt::mp("acquisition"), [this](const pmt::pmt_t &msg) {
    auto msg_key = pmt::car(msg);
    auto msg_val = pmt::cdr(msg);
    AcqResults acqResult = *(reinterpret_cast<const AcqResults *>(pmt::blob_data(msg_val)));
    if (pmt::symbol_to_string(msg_key) == "acq_restart" && !acqInProgress) {
      PRN = acqResult.PRN;
      distribute = true;
      acqInProgress = true;
      iterator = 0;
      lognSignal.clear();
      lognSignal.reserve(samplesToSend);
    } else if (pmt::symbol_to_string(msg_key) == "acq_result" && acqInProgress) {
      acqInProgress = false;
    }
  });
}

/*
 * Our virtual destructor.
 */
data_distributor_impl::~data_distributor_impl() {}

int data_distributor_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
  output_type *out = reinterpret_cast<output_type *>(output_items[0]);

  for (int i = 0; i < noutput_items; i++) {

    if (distribute && iterator < samplesToSend) {
      lognSignal.push_back(in[i]);
      iterator++;
    } else if (distribute && lognSignal.size() >= samplesToSend) {
      auto size = sizeof(gr_complex) * lognSignal.size();
      auto pmt = pmt::make_blob(lognSignal.data(), size);
      message_port_pub(pmt::string_to_symbol("data_vector"), pmt::cons(pmt::from_long(PRN), pmt));
      iterator = 0;
      lognSignal.clear();
      lognSignal.reserve(samplesToSend);
      distribute = false;
    }
    out[i] = in[i];
  }

  // Do <+signal processing+>

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
