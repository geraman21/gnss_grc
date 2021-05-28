/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include "tracking_ff_impl.h"
#include <algorithm> // std::transform

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
    {}

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

      std::transform(in, in + noutput_items, out, out, [](input_type x, input_type y) {return x * 2;});

      // Do <+signal processing+>

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace gnss */
} /* namespace gr */

