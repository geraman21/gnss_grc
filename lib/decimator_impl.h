/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_DECIMATOR_IMPL_H
#define INCLUDED_GNSS_DECIMATOR_IMPL_H

#include <gnss/decimator.h>
#include <vector>

namespace gr {
namespace gnss {

class decimator_impl : public decimator {
private:
  int test{};
  float sampleFreq;
  int decimation;
  std::vector<tag_t> tags;
  int valueIndex{};
  uint64_t absSampleCount{};

public:
  decimator_impl(float sample_freq);
  ~decimator_impl();

  // Where all the action really happens
  int work(int noutput_items, gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_DECIMATOR_IMPL_H */
