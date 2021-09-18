/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_EPHEMERIDES_IMPL_H
#define INCLUDED_GNSS_EPHEMERIDES_IMPL_H

#include "ephemeris.h"
#include <gnss/ephemerides.h>
#include <vector>

namespace gr {
namespace gnss {

class ephemerides_impl : public ephemerides {
private:
  std::vector<int> navBits;

public:
  ephemerides_impl();
  ~ephemerides_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_EPHEMERIDES_IMPL_H */
