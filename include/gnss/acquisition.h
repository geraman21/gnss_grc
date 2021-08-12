/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_ACQUISITION_H
#define INCLUDED_GNSS_ACQUISITION_H

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
    class GNSS_API acquisition : virtual public gr::sync_block
    {
    public:
      typedef std::shared_ptr<acquisition> sptr;

      /*!
   * \brief Return a shared_ptr to a new instance of gnss::acquisition.
   *
   * To avoid accidental use of raw pointers, gnss::acquisition's
   * constructor is in a private implementation
   * class. gnss::acquisition::make is the public interface for
   * creating new instances.
   */
      static sptr make(unsigned int a_sampleFreq);
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_ACQUISITION_H */
