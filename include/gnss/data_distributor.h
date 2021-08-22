/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_DATA_DISTRIBUTOR_H
#define INCLUDED_GNSS_DATA_DISTRIBUTOR_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>

namespace gr
{
  namespace gnss
  {

    /*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
    class GNSS_API data_distributor : virtual public gr::sync_block
    {
    public:
      typedef std::shared_ptr<data_distributor> sptr;

      /*!
   * \brief Return a shared_ptr to a new instance of gnss::data_distributor.
   *
   * To avoid accidental use of raw pointers, gnss::data_distributor's
   * constructor is in a private implementation
   * class. gnss::data_distributor::make is the public interface for
   * creating new instances.
   */
      static sptr make(unsigned int numSamples);
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_DATA_DISTRIBUTOR_H */
