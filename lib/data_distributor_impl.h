/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_DATA_DISTRIBUTOR_IMPL_H
#define INCLUDED_GNSS_DATA_DISTRIBUTOR_IMPL_H

#include <gnss/data_distributor.h>
#include <vector>

namespace gr {
namespace gnss {

class data_distributor_impl : public data_distributor {
private:
  unsigned long int test;
  bool distribute = true;
  unsigned int iterator = 0;
  unsigned int samplesToSend{};
  std::vector<float> lognSignal;
  unsigned long int counter{};

public:
  data_distributor_impl(float numSamples);
  ~data_distributor_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_DATA_DISTRIBUTOR_IMPL_H */
