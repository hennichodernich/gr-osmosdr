/* -*- c++ -*- */
/*
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
#ifndef FL2K_SINK_C_H
#define FL2K_SINK_C_H

#include <gnuradio/hier_block2.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/throttle.h>

#include "sink_iface.h"

#include <osmo-fl2k.h>

class fl2k_sink_c;

typedef struct circ_buf
{
  void *buffer;     // data buffer
  void *buffer_end; // end of data buffer
  size_t capacity;  // maximum number of items in the buffer
  size_t count;     // number of items in the buffer
  size_t sz;        // size of each item in the buffer
  void *head;       // pointer to head
  void *tail;       // pointer to tail
} circ_buf_t;


typedef boost::shared_ptr< fl2k_sink_c > fl2k_sink_c_sptr;

fl2k_sink_c_sptr make_fl2k_sink_c( const std::string & args = "" );

class fl2k_sink_c :
    public gr::sync_block,
    public sink_iface
{
private:
  friend fl2k_sink_c_sptr make_fl2k_sink_c(const std::string &args);

  fl2k_sink_c(const std::string &args);

public:
  ~fl2k_sink_c();

  bool start();
  bool stop();

  int work( int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items );

  std::string name();

  static std::vector< std::string > get_devices( bool fake = false );

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
  double set_gain( double gain, size_t chan = 0 );
  double set_gain( double gain, const std::string & name, size_t chan = 0 );
  double get_gain( size_t chan = 0 );
  double get_gain( const std::string & name, size_t chan = 0 );

  std::vector< std::string > get_antennas( size_t chan = 0 );
  std::string set_antenna( const std::string & antenna, size_t chan = 0 );
  std::string get_antenna( size_t chan = 0 );

  int8_t *_rbuf;
  int8_t *_gbuf;
private:
  static void _fl2k_callback(fl2k_data_info_t *data_info);
  int fl2k_callback(unsigned char *ibuffer, unsigned char *qbuffer, uint32_t length);
  static void _fl2k_wait(fl2k_sink_c *obj);
  void fl2k_wait();

  unsigned int _buf_num;
  unsigned int _buf_used;
  boost::mutex _buf_mutex;
  boost::condition_variable _buf_cond;

  fl2k_dev_t *_dev;

  double _devnum;
  double _freq, _rate;

  circ_buf_t _cbuf;

  int8_t *_buf;

};

#endif // FL2K_SINK_C_H
