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
#ifndef HNCHBOARD2U_SOURCE_C_H
#define HNCHBOARD2U_SOURCE_C_H

#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/short_to_float.h>
#include <gnuradio/hier_block2.h>
#include <iio/hnchboard2u_source.h>

#include "source_iface.h"

class hnchboard2u_source_c;

typedef std::shared_ptr< hnchboard2u_source_c > hnchboard2u_source_c_sptr;

hnchboard2u_source_c_sptr make_hnchboard2u_source_c(const std::string &args = "");

class hnchboard2u_source_c :
    public gr::hier_block2,
    public source_iface
{
private:
  friend hnchboard2u_source_c_sptr make_hnchboard2u_source_c(const std::string &args);

  hnchboard2u_source_c(const std::string &args);

public:
  ~hnchboard2u_source_c();

  static std::vector< std::string > get_devices();

  std::string name();

  size_t get_num_channels( void );

  osmosdr::meta_range_t get_sample_rates( void );
  double set_sample_rate( double rate );
  double get_sample_rate( void );

  osmosdr::freq_range_t get_freq_range( size_t chan = 0 );
  double set_center_freq( double freq, size_t chan = 0 );
  double get_center_freq( size_t chan = 0 );

  double set_freq_corr( double ppm, size_t chan = 0 );
  double get_freq_corr( size_t chan = 0 );

  std::vector<std::string> get_gain_names( size_t chan = 0 );
  osmosdr::gain_range_t get_gain_range( size_t chan = 0 );
  osmosdr::gain_range_t get_gain_range( const std::string & name, size_t chan = 0 );
  bool set_gain_mode( bool automatic, size_t chan = 0 );
  bool get_gain_mode( size_t chan = 0 );
  double set_gain( double gain, size_t chan = 0 );
  double set_gain( double gain, const std::string & name, size_t chan = 0 );
  double get_gain( size_t chan = 0 );
  double get_gain( const std::string & name, size_t chan = 0 );

  std::vector< std::string > get_antennas( size_t chan = 0 );
  std::string set_antenna( const std::string & antenna, size_t chan = 0 );
  std::string get_antenna( size_t chan = 0 );

  double set_bandwidth( double bw, size_t chan = 0 );
  double get_bandwidth( size_t chan = 0 );

private:

  void set_params(void);

  gr::iio::hnchboard2u_source::sptr       _src;

  std::string   uri;
  unsigned long samplerate;
  unsigned long lo_freq;
  unsigned long buffer_size;
  double gain_value;

};

#endif // HNCHBOARD2U_SOURCE_C_H
