# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT gnss_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT gnss_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND gnss_python_files
    tracking_ff_python.cc
    decoding_test_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(gnss 
   ../..
   gr::gnss
   "${gnss_python_files}")

install(TARGETS gnss_python DESTINATION ${GR_PYTHON_DIR}/gnss COMPONENT pythonapi)
