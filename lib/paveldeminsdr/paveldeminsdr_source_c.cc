/* -*- c++ -*- */
/*
 * Copyright 2015 Pavel Demin
 * Copyright 2012 Dimitri Stolnikov <horiz0n@gmx.net>
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

#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <gnuradio/io_signature.h>

#include "arg_helpers.h"

#include "paveldeminsdr_source_c.h"

using namespace boost::assign;

paveldeminsdr_source_c_sptr make_paveldeminsdr_source_c(const std::string &args)
{
  return gnuradio::get_initial_sptr(new paveldeminsdr_source_c(args));
}

paveldeminsdr_source_c::paveldeminsdr_source_c(const std::string &args) :
  gr::sync_block("paveldeminsdr_source_c",
                 gr::io_signature::make(0, 0, 0),
                 gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
  std::string host = "192.168.1.100";
  std::stringstream message;
  unsigned short port = 1001;
  struct sockaddr_in addr;

#if defined(_WIN32)
  WSADATA wsaData;
  WSAStartup( MAKEWORD(2, 2), &wsaData );
#endif

  _freq = 6.0e5;
  _freq_value = 600000;
  _rate = 192e3;
  _rate_value = 2;
  _corr = 0.0;

  dict_t dict = params_to_dict( args );

  if ( dict.count( "paveldeminsdr" ) )
  {
    std::vector< std::string > tokens;
    boost::algorithm::split( tokens, dict["paveldeminsdr"], boost::is_any_of( ":" ) );

    if ( tokens[0].length() && ( tokens.size() == 1 || tokens.size() == 2 ) )
      host = tokens[0];

    if ( tokens.size() == 2 )
      port = boost::lexical_cast< unsigned short >( tokens[1] );
  }

  if ( !host.length() )
    host = "192.168.1.100";

  if ( 0 == port )
    port = 1001;

  if ( ( _socket = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
    throw std::runtime_error( "Could not create TCP socket." );

  memset( &addr, 0, sizeof(addr) );
  addr.sin_family = AF_INET;
  inet_pton( AF_INET, host.c_str(), &addr.sin_addr );
  addr.sin_port = htons( port );

  if ( ::connect( _socket, (struct sockaddr *)&addr, sizeof(addr) ) < 0 )
  {
    message << "Could not connect to " << host << ":" << port << ".";
    throw std::runtime_error( message.str() );
  }
  
  _buf = (gr_complex *)malloc(BUF_SIZE_BYTES);
  if (!_buf)
  {
      message << "Could not allocate buffer.";
      throw std::runtime_error( message.str() );
  }
}

paveldeminsdr_source_c::~paveldeminsdr_source_c()
{
  free(_buf);
  
#if defined(_WIN32)
  ::closesocket( _socket );
  WSACleanup();
#else
  ::close( _socket);
#endif
}

int paveldeminsdr_source_c::work( int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items )
{
  gr_complex *out = (gr_complex *)output_items[0];
  int items_fetched;

#if defined(_WIN32)
  int size;
  int total = 8 * sizeof(gr_complex) * noutput_items;
  if (total > BUF_SIZE_BYTES)
    total = BUF_SIZE_BYTES;
  size = ::recv( _socket, (char *)_buf, total, MSG_WAITALL );
#else
  ssize_t size;
  ssize_t total = 8 * sizeof(gr_complex) * noutput_items;
  if (total > BUF_SIZE_BYTES)
    total = BUF_SIZE_BYTES;
  size = ::recv( _socket, _buf, total, MSG_WAITALL );
#endif

  if ( size != total )
    throw std::runtime_error( "Receiving samples failed." );
  
  items_fetched = (size / (8 * sizeof(gr_complex)));
  
  for(int kk=0; kk<items_fetched;++kk)
    out[kk]=_buf[8*kk];

  return items_fetched;
}

std::string paveldeminsdr_source_c::name()
{
  return "Pavel Demin SDR Rx";
}

std::vector<std::string> paveldeminsdr_source_c::get_devices( bool fake )
{
  std::vector<std::string> devices;

  if ( fake )
  {
    std::string args = "paveldeminsdr=192.168.1.100:1001";
    args += ",label='Pavel Demin SDR Receiver Server'";
    devices.push_back( args );
  }

  return devices;
}

size_t paveldeminsdr_source_c::get_num_channels( void )
{
  return 1;
}

osmosdr::meta_range_t paveldeminsdr_source_c::get_sample_rates( void )
{
  osmosdr::meta_range_t range;

  range += osmosdr::range_t( 48000 );
  range += osmosdr::range_t( 96000 );
  range += osmosdr::range_t( 192000 );
  range += osmosdr::range_t( 384000 );

  return range;
}

double paveldeminsdr_source_c::set_sample_rate( double rate )
{
  uint32_t buffer[10] = {0,0,0,0,0,0,0,0,0,0};

  if ( 48000 == rate ) _rate_value = 0;
  else if ( 96000 == rate ) _rate_value = 1;
  else if ( 192000 == rate ) _rate_value = 2;
  else if ( 384000 == rate ) _rate_value = 3;
  else return get_sample_rate();

  buffer[1] = _rate_value;
  for(int kk=2; kk<10; kk++)
    buffer[kk]=_freq_value;
  paveldeminsdr_send_commands( _socket, buffer );

  _rate = rate;

  return get_sample_rate();
}

double paveldeminsdr_source_c::get_sample_rate( void )
{
  return _rate;
}

osmosdr::freq_range_t paveldeminsdr_source_c::get_freq_range( size_t chan )
{
  return osmosdr::freq_range_t( _rate / 2.0, 6.0e7 );
}

double paveldeminsdr_source_c::set_center_freq( double freq, size_t chan )
{
  uint32_t buffer[10] = {0,0,0,0,0,0,0,0,0,0};
  
  if ( freq < _rate / 2.0 || freq > 6.0e7 ) return get_center_freq( chan );

  _freq_value = (uint32_t)floor( freq * (1.0 + _corr * 1.0e-6 ) + 0.5 );

  buffer[1] = _rate_value;
  for(int kk=2; kk<10; kk++)
    buffer[kk]=_freq_value;
  paveldeminsdr_send_commands( _socket, buffer );

  _freq = freq;

  return get_center_freq( chan );
}

double paveldeminsdr_source_c::get_center_freq( size_t chan )
{
  return _freq;
}

double paveldeminsdr_source_c::set_freq_corr( double ppm, size_t chan )
{
  _corr = ppm;

  return get_freq_corr( chan );
}

double paveldeminsdr_source_c::get_freq_corr( size_t chan )
{
  return _corr;
}

std::vector<std::string> paveldeminsdr_source_c::get_gain_names( size_t chan )
{
  return std::vector< std::string >();
}

osmosdr::gain_range_t paveldeminsdr_source_c::get_gain_range( size_t chan )
{
  return osmosdr::gain_range_t();
}

osmosdr::gain_range_t paveldeminsdr_source_c::get_gain_range( const std::string & name, size_t chan )
{
  return get_gain_range( chan );
}

double paveldeminsdr_source_c::set_gain( double gain, size_t chan )
{
  return get_gain( chan );
}

double paveldeminsdr_source_c::set_gain( double gain, const std::string & name, size_t chan )
{
  return set_gain( chan );
}

double paveldeminsdr_source_c::get_gain( size_t chan )
{
  return 0;
}

double paveldeminsdr_source_c::get_gain( const std::string & name, size_t chan )
{
  return get_gain( chan );
}

std::vector< std::string > paveldeminsdr_source_c::get_antennas( size_t chan )
{
  return std::vector< std::string >();
}

std::string paveldeminsdr_source_c::set_antenna( const std::string & antenna, size_t chan )
{
  return get_antenna( chan );
}

std::string paveldeminsdr_source_c::get_antenna( size_t chan )
{
  return "RX";
}
