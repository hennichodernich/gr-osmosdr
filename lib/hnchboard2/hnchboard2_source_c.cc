/* -*- c++ -*- */
/*
 * Copyright 2017 Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include <boost/assign.hpp>
#include <iostream>

#include "arg_helpers.h"
#include "osmosdr/source.h"
#include "hnchboard2_source_c.h"
#include <gnuradio/blocks/short_to_float.h>
#include <gnuradio/blocks/float_to_complex.h>


using namespace boost::assign;

hnchboard2_source_c_sptr make_hnchboard2_source_c(const std::string &args)
{
  return gnuradio::get_initial_sptr(new hnchboard2_source_c(args));
}

hnchboard2_source_c::hnchboard2_source_c(const std::string &args) :
    gr::hier_block2("hnchboard2_source_c",
                   gr::io_signature::make(0, 0, 0),
                   gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
  gr::blocks::short_to_float::sptr s2f1 =  gr::blocks::short_to_float::make(1, 2048.0);
  gr::blocks::short_to_float::sptr s2f2 =  gr::blocks::short_to_float::make(1, 2048.0);
  gr::blocks::float_to_complex::sptr f2c =  gr::blocks::float_to_complex::make(1);

  uri = "ip:hnchboard2.local";
  samplerate = 5000000;
  lo_freq = 100000000;
  buffer_size = 0x10000;
  gain_value = 4.5;
  
  dict_t dict = params_to_dict(args);
  if (dict.count("uri"))
    uri = boost::lexical_cast< std::string >( dict["uri"] );

  std::cerr << "Using hnchBoard2 URI = " << uri << std::endl;

  _src = gr::iio::hnchboard2_source::make(uri, samplerate, lo_freq, gain_value, buffer_size);  
  
  connect(_src, 0, s2f1, 0);
  connect(_src, 1, s2f2, 0);
  connect(s2f1, 0, f2c, 0);
  connect(s2f2, 0, f2c, 1);
  connect(f2c, 0, self(), 0);    
}

hnchboard2_source_c::~hnchboard2_source_c()
{
}

std::vector< std::string > hnchboard2_source_c::get_devices()
{
  std::vector< std::string > devices;

  std::string args = "hnchboard2,label='hnchBoard2'";

  devices.push_back( args );

  return devices;
}

std::string hnchboard2_source_c::name()
{
  return "hnchBoard2";
}

size_t hnchboard2_source_c::get_num_channels()
{
  return output_signature()->max_streams();
}

osmosdr::meta_range_t hnchboard2_source_c::get_sample_rates( void )
{
  osmosdr::meta_range_t rates;

  rates += osmosdr::range_t(   500000 );    
  rates += osmosdr::range_t(  1000000 );
  rates += osmosdr::range_t(  2500000 );
  rates += osmosdr::range_t(  3000000 );    
  rates += osmosdr::range_t(  5000000 );    
  rates += osmosdr::range_t( 10000000 );
  rates += osmosdr::range_t( 20000000 );    
  rates += osmosdr::range_t( 40000000 );  

  return rates;
}

double hnchboard2_source_c::set_sample_rate( double rate )
{
  samplerate = (unsigned long) rate;
  set_params();

  return samplerate;
}

double hnchboard2_source_c::get_sample_rate( void )
{
  return samplerate;
}

osmosdr::freq_range_t hnchboard2_source_c::get_freq_range( size_t chan )
{
  osmosdr::freq_range_t range;

  range += osmosdr::range_t( 70000000, 500000000, 1.0 );

  return range;
}

double hnchboard2_source_c::set_center_freq( double freq, size_t chan )
{
  lo_freq = (unsigned long) freq;
  set_params();
  return (double)lo_freq;
}

double hnchboard2_source_c::get_center_freq( size_t chan )
{
  return (double)lo_freq;
}

double hnchboard2_source_c::set_freq_corr( double ppm, size_t chan)
{
  return 0;
}

double hnchboard2_source_c::get_freq_corr( size_t chan)
{
  return 0;
}

std::vector<std::string> hnchboard2_source_c::get_gain_names( size_t chan )
{
  std::vector< std::string > gains;
  gains.push_back( "Dummy" );
  
  return gains;
}

osmosdr::gain_range_t hnchboard2_source_c::get_gain_range( size_t chan)
{
  osmosdr::gain_range_t range;
  range += osmosdr::range_t(4.5, 20.25, 0.253 );

  return range;
}

osmosdr::gain_range_t hnchboard2_source_c::get_gain_range( const std::string & name,
                                                         size_t chan)
{
  osmosdr::gain_range_t range;

    range += osmosdr::range_t(4.5, 20.25, 0.253 );

  return range;
}

bool hnchboard2_source_c::set_gain_mode( bool automatic, size_t chan )
{
  return 0;
}

bool hnchboard2_source_c::get_gain_mode( size_t chan )
{
  return 0;
}

double hnchboard2_source_c::set_gain( double gain, size_t chan )
{  
    gain_value = gain;
    set_params();
    return 0;
}

double hnchboard2_source_c::set_gain( double gain, const std::string & name, size_t chan )
{
    gain_value = gain;
    set_params();
    return 0;
}

double hnchboard2_source_c::get_gain( size_t chan )
{
  return gain_value;
}

double hnchboard2_source_c::get_gain( const std::string & name, size_t chan )
{
  return gain_value;
}

std::vector< std::string > hnchboard2_source_c::get_antennas( size_t chan )
{
  std::vector< std::string > antennas;

  antennas += get_antenna( chan );

  return antennas;
}

std::string hnchboard2_source_c::set_antenna( const std::string & antenna, size_t chan )
{
  return get_antenna( chan );
}

std::string hnchboard2_source_c::get_antenna( size_t chan )
{
  return "Default";
}

double hnchboard2_source_c::set_bandwidth( double bw, size_t chan )
{
    
  return (5000000);
}

double hnchboard2_source_c::get_bandwidth( size_t chan )
{
  return (5000000);
}

void hnchboard2_source_c::set_params( void )
{  
  _src->set_params(samplerate, lo_freq, gain_value);
  
  return;
}
