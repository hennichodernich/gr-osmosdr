/*
 * Copyright 2020 Free Software Foundation, Inc.
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
/* BINDTOOL_HEADER_FILE(source.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(d1a3d9ea3d815fe4f18acc3eef21f1b6)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <osmosdr/source.h>
// pydoc.h is automatically generated in the build directory
#include <source_pydoc.h>

void bind_source(py::module& m)
{

    using source    = ::osmosdr::source;


    py::class_<source, gr::hier_block2,
        std::shared_ptr<source>>(m, "source", D(source))

        .def(py::init(&source::make),
           py::arg("args") = "",
           D(source,make)
        )
        




        .def("get_num_channels",&source::get_num_channels,
            D(source,get_num_channels)
        )


        .def("seek",&source::seek,
            py::arg("seek_point"),
            py::arg("whence"),
            py::arg("chan") = 0,
            D(source,seek)
        )


        .def("get_sample_rates",&source::get_sample_rates,
            D(source,get_sample_rates)
        )


        .def("set_sample_rate",&source::set_sample_rate,
            py::arg("rate"),
            D(source,set_sample_rate)
        )


        .def("get_sample_rate",&source::get_sample_rate,
            D(source,get_sample_rate)
        )


        .def("get_freq_range",&source::get_freq_range,
            py::arg("chan") = 0,
            D(source,get_freq_range)
        )


        .def("set_center_freq",&source::set_center_freq,
            py::arg("freq"),
            py::arg("chan") = 0,
            D(source,set_center_freq)
        )


        .def("get_center_freq",&source::get_center_freq,
            py::arg("chan") = 0,
            D(source,get_center_freq)
        )


        .def("set_freq_corr",&source::set_freq_corr,
            py::arg("ppm"),
            py::arg("chan") = 0,
            D(source,set_freq_corr)
        )


        .def("get_freq_corr",&source::get_freq_corr,
            py::arg("chan") = 0,
            D(source,get_freq_corr)
        )


        .def("get_gain_names",&source::get_gain_names,
            py::arg("chan") = 0,
            D(source,get_gain_names)
        )


        .def("get_gain_range",(osmosdr::gain_range_t (source::*)(size_t))&source::get_gain_range,
            py::arg("chan") = 0,
            D(source,get_gain_range,0)
        )


        .def("get_gain_range",(osmosdr::gain_range_t (source::*)(std::string const &, size_t))&source::get_gain_range,
            py::arg("name"),
            py::arg("chan") = 0,
            D(source,get_gain_range,1)
        )


        .def("set_gain_mode",&source::set_gain_mode,
            py::arg("automatic"),
            py::arg("chan") = 0,
            D(source,set_gain_mode)
        )


        .def("get_gain_mode",&source::get_gain_mode,
            py::arg("chan") = 0,
            D(source,get_gain_mode)
        )


        .def("set_gain",(double (source::*)(double, size_t))&source::set_gain,
            py::arg("gain"),
            py::arg("chan") = 0,
            D(source,set_gain,0)
        )


        .def("set_gain",(double (source::*)(double, std::string const &, size_t))&source::set_gain,
            py::arg("gain"),
            py::arg("name"),
            py::arg("chan") = 0,
            D(source,set_gain,1)
        )


        .def("get_gain",(double (source::*)(size_t))&source::get_gain,
            py::arg("chan") = 0,
            D(source,get_gain,0)
        )


        .def("get_gain",(double (source::*)(std::string const &, size_t))&source::get_gain,
            py::arg("name"),
            py::arg("chan") = 0,
            D(source,get_gain,1)
        )


        .def("set_if_gain",&source::set_if_gain,
            py::arg("gain"),
            py::arg("chan") = 0,
            D(source,set_if_gain)
        )


        .def("set_bb_gain",&source::set_bb_gain,
            py::arg("gain"),
            py::arg("chan") = 0,
            D(source,set_bb_gain)
        )


        .def("get_antennas",&source::get_antennas,
            py::arg("chan") = 0,
            D(source,get_antennas)
        )


        .def("set_antenna",&source::set_antenna,
            py::arg("antenna"),
            py::arg("chan") = 0,
            D(source,set_antenna)
        )


        .def("get_antenna",&source::get_antenna,
            py::arg("chan") = 0,
            D(source,get_antenna)
        )


        .def("set_dc_offset_mode",&source::set_dc_offset_mode,
            py::arg("mode"),
            py::arg("chan") = 0,
            D(source,set_dc_offset_mode)
        )


        .def("set_dc_offset",&source::set_dc_offset,
            py::arg("offset"),
            py::arg("chan") = 0,
            D(source,set_dc_offset)
        )


        .def("set_iq_balance_mode",&source::set_iq_balance_mode,
            py::arg("mode"),
            py::arg("chan") = 0,
            D(source,set_iq_balance_mode)
        )


        .def("set_iq_balance",&source::set_iq_balance,
            py::arg("balance"),
            py::arg("chan") = 0,
            D(source,set_iq_balance)
        )


        .def("set_bandwidth",&source::set_bandwidth,
            py::arg("bandwidth"),
            py::arg("chan") = 0,
            D(source,set_bandwidth)
        )


        .def("get_bandwidth",&source::get_bandwidth,
            py::arg("chan") = 0,
            D(source,get_bandwidth)
        )


        .def("get_bandwidth_range",&source::get_bandwidth_range,
            py::arg("chan") = 0,
            D(source,get_bandwidth_range)
        )


        .def("set_time_source",&source::set_time_source,
            py::arg("source"),
            py::arg("mboard") = 0,
            D(source,set_time_source)
        )


        .def("get_time_source",&source::get_time_source,
            py::arg("mboard"),
            D(source,get_time_source)
        )


        .def("get_time_sources",&source::get_time_sources,
            py::arg("mboard"),
            D(source,get_time_sources)
        )


        .def("set_clock_source",&source::set_clock_source,
            py::arg("source"),
            py::arg("mboard") = 0,
            D(source,set_clock_source)
        )


        .def("get_clock_source",&source::get_clock_source,
            py::arg("mboard"),
            D(source,get_clock_source)
        )


        .def("get_clock_sources",&source::get_clock_sources,
            py::arg("mboard"),
            D(source,get_clock_sources)
        )


        .def("get_clock_rate",&source::get_clock_rate,
            py::arg("mboard") = 0,
            D(source,get_clock_rate)
        )


        .def("set_clock_rate",&source::set_clock_rate,
            py::arg("rate"),
            py::arg("mboard") = 0,
            D(source,set_clock_rate)
        )


        .def("get_time_now",&source::get_time_now,
            py::arg("mboard") = 0,
            D(source,get_time_now)
        )


        .def("get_time_last_pps",&source::get_time_last_pps,
            py::arg("mboard") = 0,
            D(source,get_time_last_pps)
        )


        .def("set_time_now",&source::set_time_now,
            py::arg("time_spec"),
            py::arg("mboard") = 0,
            D(source,set_time_now)
        )


        .def("set_time_next_pps",&source::set_time_next_pps,
            py::arg("time_spec"),
            D(source,set_time_next_pps)
        )


        .def("set_time_unknown_pps",&source::set_time_unknown_pps,
            py::arg("time_spec"),
            D(source,set_time_unknown_pps)
        )

        ;




}







