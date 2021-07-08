/* -*- c++ -*- */
/*
 * Copyright 2021 German Abdurahmanov.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_GNSS_NAV_SOLUTION_IMPL_H
#define INCLUDED_GNSS_NAV_SOLUTION_IMPL_H

#include <gnss/nav_solution.h>

namespace gr {
namespace gnss {

class nav_solution_impl : public nav_solution
{
private:
    // Nothing to declare in this block.

public:
    nav_solution_impl();
    ~nav_solution_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_SOLUTION_IMPL_H */
