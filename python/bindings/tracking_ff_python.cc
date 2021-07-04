/*
 * Copyright 2021 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(tracking_ff.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(28c3247e1c7b885c72ada1513f5b3c41)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnss/tracking_ff.h>
// pydoc.h is automatically generated in the build directory
#include <tracking_ff_pydoc.h>

void bind_tracking_ff(py::module& m)
{

    using tracking_ff = gr::gnss::tracking_ff;


    py::class_<tracking_ff,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<tracking_ff>>(m, "tracking_ff", D(tracking_ff))

        .def(py::init(&tracking_ff::make),
             py::arg("t_prn"),
             py::arg("t_freq"),
             py::arg("t_codePhase"),
             D(tracking_ff, make))


        ;
}
