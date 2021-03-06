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
/* BINDTOOL_HEADER_FILE(channel_starter.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(dfa75ca1bb847ef384ebe3efb7bd44a2)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnss/channel_starter.h>
// pydoc.h is automatically generated in the build directory
#include <channel_starter_pydoc.h>

void bind_channel_starter(py::module &m) {

  using channel_starter = gr::gnss::channel_starter;

  py::class_<channel_starter, gr::block, gr::basic_block, std::shared_ptr<channel_starter>>(
      m, "channel_starter", D(channel_starter))

      .def(py::init(&channel_starter::make), py::arg("attempts"), py::arg("s_sampleFreq"),
           py::arg("im_freq"), D(channel_starter, make))

      ;
}
