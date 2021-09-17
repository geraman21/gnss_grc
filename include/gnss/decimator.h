/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_DECIMATOR_H
#define INCLUDED_GNSS_DECIMATOR_H

#include <gnss/api.h>
#include <gnuradio/sync_decimator.h>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API decimator : virtual public gr::sync_decimator {
public:
  typedef std::shared_ptr<decimator> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of gnss::decimator.
   *
   * To avoid accidental use of raw pointers, gnss::decimator's
   * constructor is in a private implementation
   * class. gnss::decimator::make is the public interface for
   * creating new instances.
   */
  static sptr make(float sample_freq);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_DECIMATOR_H */
