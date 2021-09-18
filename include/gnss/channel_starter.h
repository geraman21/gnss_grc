/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_CHANNEL_STARTER_H
#define INCLUDED_GNSS_CHANNEL_STARTER_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API channel_starter : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<channel_starter> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of gnss::channel_starter.
   *
   * To avoid accidental use of raw pointers, gnss::channel_starter's
   * constructor is in a private implementation
   * class. gnss::channel_starter::make is the public interface for
   * creating new instances.
   */
  static sptr make(int attempts, float s_sampleFreq);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_CHANNEL_STARTER_H */
