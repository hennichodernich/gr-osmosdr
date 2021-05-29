/* -*- c++ -*- */
/*
 * Copyright 2013 Dimitri Stolnikov <horiz0n@gmx.net>
 * Copyright 2020 Clayton Smith <argilo@gmail.com>
 *
 * gr-osmosdr is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * gr-osmosdr is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gr-osmosdr; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/*
 * config.h is generated by configure.  It contains the results
 * of probing for features, options etc.  It should be the first
 * file included in your .cc file.
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdexcept>
#include <iostream>

#include <gnuradio/io_signature.h>

#include "hackrf_source_c.h"

#include "arg_helpers.h"

hackrf_source_c_sptr make_hackrf_source_c (const std::string & args)
{
  return gnuradio::get_initial_sptr(new hackrf_source_c (args));
}

/*
 * Specify constraints on number of input and output streams.
 * This info is used to construct the input and output signatures
 * (2nd & 3rd args to gr::block's constructor).  The input and
 * output signatures are used by the runtime system to
 * check that a valid number and type of inputs and outputs
 * are connected to this block.  In this case, we accept
 * only 0 input and 1 output.
 */
static const int MIN_IN = 0;	// mininum number of input streams
static const int MAX_IN = 0;	// maximum number of input streams
static const int MIN_OUT = 1;	// minimum number of output streams
static const int MAX_OUT = 1;	// maximum number of output streams

/*
 * The private constructor
 */
hackrf_source_c::hackrf_source_c (const std::string &args)
  : gr::sync_block ("hackrf_source_c",
        gr::io_signature::make(MIN_IN, MAX_IN, sizeof (gr_complex)),
        gr::io_signature::make(MIN_OUT, MAX_OUT, sizeof (gr_complex))),
    hackrf_common::hackrf_common(args),
    _buf(NULL),
    _lna_gain(0),
    _vga_gain(0)
{
  dict_t dict = params_to_dict(args);

  _buf_num = _buf_len = _buf_head = _buf_used = _buf_offset = 0;

  if (dict.count("buffers"))
    _buf_num = std::stoi(dict["buffers"]);

//  if (dict.count("buflen"))
//    _buf_len = std::stoi(dict["buflen"]);

  if (0 == _buf_num)
    _buf_num = BUF_NUM;

  if (0 == _buf_len || _buf_len % 512 != 0) /* len must be multiple of 512 */
    _buf_len = BUF_LEN;

  _samp_avail = _buf_len / BYTES_PER_SAMPLE;

  // create a lookup table for gr_complex values
  for (unsigned int i = 0; i <= 0xff; i++) {
    _lut.push_back( float(int8_t(i)) * (1.0f/128.0f) );
  }

  if ( BUF_NUM != _buf_num || BUF_LEN != _buf_len ) {
    std::cerr << "Using " << _buf_num << " buffers of size " << _buf_len << "."
              << std::endl;
  }

  set_center_freq( (get_freq_range().start() + get_freq_range().stop()) / 2.0 );
  set_sample_rate( get_sample_rates().start() );
  set_bandwidth( 0 );

  set_gain( 0 ); /* disable AMP gain stage by default to protect full sprectrum pre-amp from physical damage */

  set_if_gain( 16 ); /* preset to a reasonable default (non-GRC use case) */

  set_bb_gain( 20 ); /* preset to a reasonable default (non-GRC use case) */

  // Check device args to find out if bias/phantom power is desired.
  if ( dict.count("bias") ) {
    hackrf_common::set_bias(dict["bias"] == "1");
  }

  _buf = (unsigned char **) malloc(_buf_num * sizeof(unsigned char *));

  if (_buf) {
    for(unsigned int i = 0; i < _buf_num; ++i)
      _buf[i] = (unsigned char *) malloc(_buf_len);
  }
}

/*
 * Our virtual destructor.
 */
hackrf_source_c::~hackrf_source_c ()
{
  if (_buf) {
    for(unsigned int i = 0; i < _buf_num; ++i) {
      free(_buf[i]);
    }

    free(_buf);
    _buf = NULL;
  }
}

int hackrf_source_c::_hackrf_rx_callback(hackrf_transfer *transfer)
{
  hackrf_source_c *obj = (hackrf_source_c *)transfer->rx_ctx;
  return obj->hackrf_rx_callback(transfer->buffer, transfer->valid_length);
}

int hackrf_source_c::hackrf_rx_callback(unsigned char *buf, uint32_t len)
{
  {
    std::lock_guard<std::mutex> lock(_buf_mutex);

    int buf_tail = (_buf_head + _buf_used) % _buf_num;
    memcpy(_buf[buf_tail], buf, len);

    if (_buf_used == _buf_num) {
      std::cerr << "O" << std::flush;
      _buf_head = (_buf_head + 1) % _buf_num;
    } else {
      _buf_used++;
    }
  }

  _buf_cond.notify_one();

  return 0; // TODO: return -1 on error/stop
}

bool hackrf_source_c::start()
{
  if ( ! _dev.get() )
    return false;

  hackrf_common::start();
  int ret = hackrf_start_rx( _dev.get(), _hackrf_rx_callback, (void *)this );
  if ( ret != HACKRF_SUCCESS ) {
    std::cerr << "Failed to start RX streaming (" << ret << ")" << std::endl;
    return false;
  }
  return true;
}

bool hackrf_source_c::stop()
{
  if ( ! _dev.get() )
    return false;

  hackrf_common::stop();
  int ret = hackrf_stop_rx( _dev.get() );
  if ( ret != HACKRF_SUCCESS ) {
    std::cerr << "Failed to stop RX streaming (" << ret << ")" << std::endl;
    return false;
  }
  return true;
}

int hackrf_source_c::work( int noutput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items )
{
  gr_complex *out = (gr_complex *)output_items[0];

  bool running = false;

  if ( _dev.get() )
    running = (hackrf_is_streaming( _dev.get() ) == HACKRF_TRUE);

  {
    std::unique_lock<std::mutex> lock(_buf_mutex);

    while (_buf_used < 3 && running) // collect at least 3 buffers
      _buf_cond.wait( lock );
  }

  if ( ! running )
    return WORK_DONE;

  const uint8_t *buf = _buf[_buf_head] + _buf_offset * BYTES_PER_SAMPLE;
#define TO_COMPLEX(p) gr_complex( _lut[(p)[0]], _lut[(p)[1]] )

  if (noutput_items <= _samp_avail) {
    for (int i = 0; i < noutput_items; ++i)
      *out++ = TO_COMPLEX( buf + i*BYTES_PER_SAMPLE );

    _buf_offset += noutput_items;
    _samp_avail -= noutput_items;
  } else {
    for (int i = 0; i < _samp_avail; ++i)
      *out++ = TO_COMPLEX( buf + i*BYTES_PER_SAMPLE );

    {
      std::lock_guard<std::mutex> lock(_buf_mutex);

      _buf_head = (_buf_head + 1) % _buf_num;
      _buf_used--;
    }

    buf = _buf[_buf_head];

    int remaining = noutput_items - _samp_avail;

    for (int i = 0; i < remaining; ++i)
      *out++ = TO_COMPLEX( buf + i*BYTES_PER_SAMPLE );

    _buf_offset = remaining;
    _samp_avail = (_buf_len / BYTES_PER_SAMPLE) - remaining;
  }

  return noutput_items;
}

std::vector<std::string> hackrf_source_c::get_devices()
{
  return hackrf_common::get_devices();
}

size_t hackrf_source_c::get_num_channels()
{
  return 1;
}

osmosdr::meta_range_t hackrf_source_c::get_sample_rates()
{
  return hackrf_common::get_sample_rates();
}

double hackrf_source_c::set_sample_rate( double rate )
{
  return hackrf_common::set_sample_rate(rate);
}

double hackrf_source_c::get_sample_rate()
{
  return hackrf_common::get_sample_rate();
}

osmosdr::freq_range_t hackrf_source_c::get_freq_range( size_t chan )
{
  return hackrf_common::get_freq_range(chan);
}

double hackrf_source_c::set_center_freq( double freq, size_t chan )
{
  return hackrf_common::set_center_freq(freq, chan);
}

double hackrf_source_c::get_center_freq( size_t chan )
{
  return hackrf_common::get_center_freq(chan);
}

double hackrf_source_c::set_freq_corr( double ppm, size_t chan )
{
  return hackrf_common::set_freq_corr(ppm, chan);
}

double hackrf_source_c::get_freq_corr( size_t chan )
{
  return hackrf_common::get_freq_corr(chan);
}

std::vector<std::string> hackrf_source_c::get_gain_names( size_t chan )
{
  return { "RF", "IF", "BB" };
}

osmosdr::gain_range_t hackrf_source_c::get_gain_range( size_t chan )
{
  return get_gain_range( "RF", chan );
}

osmosdr::gain_range_t hackrf_source_c::get_gain_range( const std::string & name, size_t chan )
{
  if ( "RF" == name ) {
    return osmosdr::gain_range_t( 0, 14, 14 );
  }

  if ( "IF" == name ) {
    return osmosdr::gain_range_t( 0, 40, 8 );
  }

  if ( "BB" == name ) {
    return osmosdr::gain_range_t( 0, 62, 2 );
  }

  return osmosdr::gain_range_t();
}

bool hackrf_source_c::set_gain_mode( bool automatic, size_t chan )
{
  return hackrf_common::set_gain_mode(automatic, chan);
}

bool hackrf_source_c::get_gain_mode( size_t chan )
{
  return hackrf_common::get_gain_mode(chan);
}

double hackrf_source_c::set_gain( double gain, size_t chan )
{
  return hackrf_common::set_gain(gain, chan);
}

double hackrf_source_c::set_gain( double gain, const std::string & name, size_t chan)
{
  if ( "RF" == name ) {
    return set_gain( gain, chan );
  }

  if ( "IF" == name ) {
    return set_if_gain( gain, chan );
  }

  if ( "BB" == name ) {
    return set_bb_gain( gain, chan );
  }

  return set_gain( gain, chan );
}

double hackrf_source_c::get_gain( size_t chan )
{
  return hackrf_common::get_gain(chan);
}

double hackrf_source_c::get_gain( const std::string & name, size_t chan )
{
  if ( "RF" == name ) {
    return get_gain( chan );
  }

  if ( "IF" == name ) {
    return _lna_gain;
  }

  if ( "BB" == name ) {
    return _vga_gain;
  }

  return get_gain( chan );
}

double hackrf_source_c::set_if_gain(double gain, size_t chan)
{
  int ret;
  osmosdr::gain_range_t rf_gains = get_gain_range( "IF", chan );

  if (_dev.get()) {
    double clip_gain = rf_gains.clip( gain, true );

    ret = hackrf_set_lna_gain( _dev.get(), uint32_t(clip_gain) );
    if ( HACKRF_SUCCESS == ret ) {
      _lna_gain = clip_gain;
    } else {
      HACKRF_THROW_ON_ERROR( ret, HACKRF_FUNC_STR( "hackrf_set_lna_gain", clip_gain ) )
    }
  }

  return _lna_gain;
}

double hackrf_source_c::set_bb_gain( double gain, size_t chan )
{
  int ret;
  osmosdr::gain_range_t if_gains = get_gain_range( "BB", chan );

  if (_dev.get()) {
    double clip_gain = if_gains.clip( gain, true );

    ret = hackrf_set_vga_gain( _dev.get(), uint32_t(clip_gain) );
    if ( HACKRF_SUCCESS == ret ) {
      _vga_gain = clip_gain;
    } else {
      HACKRF_THROW_ON_ERROR( ret, HACKRF_FUNC_STR( "hackrf_set_vga_gain", clip_gain ) )
    }
  }

  return _vga_gain;
}

std::vector< std::string > hackrf_source_c::get_antennas( size_t chan )
{
  return hackrf_common::get_antennas(chan);
}

std::string hackrf_source_c::set_antenna( const std::string & antenna, size_t chan )
{
  return hackrf_common::set_antenna(antenna, chan);
}

std::string hackrf_source_c::get_antenna( size_t chan )
{
  return hackrf_common::get_antenna(chan);
}

double hackrf_source_c::set_bandwidth( double bandwidth, size_t chan )
{
  return hackrf_common::set_bandwidth(bandwidth, chan);
}

double hackrf_source_c::get_bandwidth( size_t chan )
{
  return hackrf_common::get_bandwidth(chan);
}

osmosdr::freq_range_t hackrf_source_c::get_bandwidth_range( size_t chan )
{
  return hackrf_common::get_bandwidth_range(chan);
}
