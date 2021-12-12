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
/* BINDTOOL_GEN_AUTOMATIC(1)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(nav_decoding.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(f669570491c422c91c473b1e2880390d)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnss/nav_decoding.h>
// pydoc.h is automatically generated in the build directory
#include <nav_decoding_pydoc.h>

void bind_nav_decoding(py::module &m) {

  using nav_decoding = gr::gnss::nav_decoding;

  py::class_<nav_decoding, gr::sync_block, gr::block, gr::basic_block,
             std::shared_ptr<nav_decoding>>(m, "nav_decoding", D(nav_decoding))

      .def(py::init(&nav_decoding::make), py::arg("channelNum"), py::arg("_sampleFreq"),
           D(nav_decoding, make))

      ;
}
