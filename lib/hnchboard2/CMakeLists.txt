# Copyright 2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# This file included, use CMake directory variables
########################################################################

target_include_directories(gnuradio-osmosdr PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GNURADIO_IIO_INCLUDE_DIRS}
    ${IIO_INCLUDE_DIRS}
)

APPEND_LIB_LIST(
	gnuradio::gnuradio-iio
	${IIO_LIBRARIES}
)

list(APPEND gr_osmosdr_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/hnchboard2_source_c.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/hnchboard2u_source_c.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/hnchbbboard_source_c.cc
)
set(gr_osmosdr_srcs ${gr_osmosdr_srcs} PARENT_SCOPE)

