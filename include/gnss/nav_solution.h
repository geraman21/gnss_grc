/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_NAV_SOLUTION_H
#define INCLUDED_GNSS_NAV_SOLUTION_H

#include <gnss/api.h>
#include <gnuradio/sync_decimator.h>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API nav_solution : virtual public gr::sync_decimator {
public:
  typedef std::shared_ptr<nav_solution> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of gnss::nav_solution.
   *
   * To avoid accidental use of raw pointers, gnss::nav_solution's
   * constructor is in a private implementation
   * class. gnss::nav_solution::make is the public interface for
   * creating new instances.
   */
  static sptr make(float _sampleFreq, int _updateRate);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_SOLUTION_H */
