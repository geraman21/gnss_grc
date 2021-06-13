/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_DECODING_TEST_H
#define INCLUDED_GNSS_DECODING_TEST_H

#include <gnuradio/sync_block.h>
#include <gnss/api.h>
#include <vector>

namespace gr {
namespace gnss {

/*!
 * \brief <+description of block+>
 * \ingroup gnss
 *
 */
class GNSS_API decoding_test : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<decoding_test> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of gnss::decoding_test.
     *
     * To avoid accidental use of raw pointers, gnss::decoding_test's
     * constructor is in a private implementation
     * class. gnss::decoding_test::make is the public interface for
     * creating new instances.
     */
    static sptr make(int prn);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_DECODING_TEST_H */
