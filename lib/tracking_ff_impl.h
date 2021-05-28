/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_TRACKING_FF_IMPL_H
#define INCLUDED_GNSS_TRACKING_FF_IMPL_H

#include <gnss/tracking_ff.h>

namespace gr {
  namespace gnss {

    class tracking_ff_impl : public tracking_ff
    {
     private:
      // Nothing to declare in this block.

     public:
      tracking_ff_impl();
      ~tracking_ff_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_TRACKING_FF_IMPL_H */

