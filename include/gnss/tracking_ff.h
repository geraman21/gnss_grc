/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_TRACKING_FF_H
#define INCLUDED_GNSS_TRACKING_FF_H

#include <gnuradio/sync_block.h>
#include <gnss/api.h>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API tracking_ff : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<tracking_ff> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of gnss::tracking_ff.
     *
     * To avoid accidental use of raw pointers, gnss::tracking_ff's
     * constructor is in a private implementation
     * class. gnss::tracking_ff::make is the public interface for
     * creating new instances.
     */
    static sptr make(int t_prn, float t_freq, int t_codePhase);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_TRACKING_FF_H */
