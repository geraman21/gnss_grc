/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ephemerides_impl.h"
#include "ephemeris.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {

using input_type = float;
ephemerides::sptr ephemerides::make() { return gnuradio::make_block_sptr<ephemerides_impl>(); }

/*
 * The private constructor
 */
ephemerides_impl::ephemerides_impl()
    : gr::sync_block(
          "ephemerides",
          gr::io_signature::make(0 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(0, 0, 0)) {
  message_port_register_in(pmt::string_to_symbol("nav_bits"));
  message_port_register_out(pmt::string_to_symbol("ephemeris"));

  set_msg_handler(pmt::mp("nav_bits"), [this](const pmt::pmt_t &msg) {
    auto msg_key = pmt::car(msg);
    auto msg_val = pmt::cdr(msg);
    int channel = pmt::to_long(msg_key);

    navBits.clear();
    navBits.reserve(1501);
    const int *data = reinterpret_cast<const int *>(pmt::blob_data(msg_val));
    navBits.assign(data, data + navBits.capacity());
    Ephemeris ephResults(navBits, channel);
    auto size = sizeof(Ephemeris);
    auto pmt = pmt::make_blob(reinterpret_cast<void *>(&ephResults), size);
    message_port_pub(pmt::string_to_symbol("ephemeris"), pmt);
  });
}

/*
 * Our virtual destructor.
 */
ephemerides_impl::~ephemerides_impl() {}

int ephemerides_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);

  // Do <+signal processing+>

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
