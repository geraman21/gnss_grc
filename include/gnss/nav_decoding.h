/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_NAV_DECODING_H
#define INCLUDED_GNSS_NAV_DECODING_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API nav_decoding : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<nav_decoding> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of gnss::nav_decoding.
   *
   * To avoid accidental use of raw pointers, gnss::nav_decoding's
   * constructor is in a private implementation
   * class. gnss::nav_decoding::make is the public interface for
   * creating new instances.
   */
  static sptr make(int channelNum, float _sampleFreq);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_DECODING_H */
