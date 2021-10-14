/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "decimator_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace gnss {

using input_type = float;
using output_type = float;
decimator::sptr decimator::make(float sample_freq) {
  return gnuradio::make_block_sptr<decimator_impl>(sample_freq);
}

/*
 * The private constructor
 */
decimator_impl::decimator_impl(float sample_freq)
    : gr::sync_decimator(
          "decimator",
          gr::io_signature::make(1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
          gr::io_signature::make(1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)),
          std::ceil(sample_freq / 1000) /*<+decimation+>*/),
      sampleFreq{sample_freq} {
  set_tag_propagation_policy(TPP_DONT);
  decimation = std::ceil(sampleFreq / 1000);
}

/*
 * Our virtual destructor.
 */
decimator_impl::~decimator_impl() {}

int decimator_impl::work(int noutput_items, gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items) {
  const input_type *in = reinterpret_cast<const input_type *>(input_items[0]);
  output_type *out = reinterpret_cast<output_type *>(output_items[0]);

  const uint64_t nread = this->nitems_read(0); // number of items read on port 0
  const size_t ninput_items = noutput_items * decimation;
  // read all tags associated with port 0 for items in this work function
  tags.clear();
  this->get_tags_in_range(tags, 0, nread, nread + ninput_items);

  for (int j = 0; j < noutput_items; j++) {
    for (int i = j * decimation; i < j * decimation + decimation; i++) {
      if (in[i] != 0) {
        valueIndex = i;
      }
    }
    if (tags.size() == noutput_items) {
      tag_t tag;
      tag.offset = this->nitems_written(0) + j;
      tag.key = tags.at(j).key;
      tag.value = tags.at(j).value;
      this->add_item_tag(0, tag);
    }
    out[j] = in[valueIndex];
  }

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

} /* namespace gnss */
} /* namespace gr */
