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
/* BINDTOOL_HEADER_FILE(nav_solution.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(06ba248b30fff057840765e472cd0673)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnss/nav_solution.h>
// pydoc.h is automatically generated in the build directory
#include <nav_solution_pydoc.h>

void bind_nav_solution(py::module &m) {

  using nav_solution = gr::gnss::nav_solution;

  py::class_<nav_solution, gr::sync_decimator, std::shared_ptr<nav_solution>>(m, "nav_solution",
                                                                              D(nav_solution))

      .def(py::init(&nav_solution::make), py::arg("_sampleFreq"), py::arg("_updateRate"),
           D(nav_solution, make))

      ;
}
