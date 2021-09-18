/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_EPHEMERIDES_H
#define INCLUDED_GNSS_EPHEMERIDES_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API ephemerides : virtual public gr::sync_block {
public:
  typedef std::shared_ptr<ephemerides> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of gnss::ephemerides.
   *
   * To avoid accidental use of raw pointers, gnss::ephemerides's
   * constructor is in a private implementation
   * class. gnss::ephemerides::make is the public interface for
   * creating new instances.
   */
  static sptr make();
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_EPHEMERIDES_H */
