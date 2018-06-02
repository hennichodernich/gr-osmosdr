/* -*- c++ -*- */
/*
 * Copyright 2012 Dimitri Stolnikov <horiz0n@gmx.net>
 * Copyright 2014 Hoernchen <la@tfc-server.de>
 * Copyright 2018 Henning Paul <hnch@gmx.net>
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

#ifdef USE_AVX
#include <immintrin.h>
#elif USE_SSE2
#include <emmintrin.h>
#endif

#include <boost/assign.hpp>
#include <boost/format.hpp>

#include <gnuradio/io_signature.h>

#include "fl2k_sink_c.h"

#include "arg_helpers.h"

using namespace boost::assign;


#define BUF_LEN  2*FL2K_BUF_LEN
#define BUF_NUM	15

static const int MIN_IN = 1;  // mininum number of input streams
static const int MAX_IN = 1;  // maximum number of input streams
static const int MIN_OUT = 0;  // minimum number of output streams
static const int MAX_OUT = 0;  // maximum number of output streams

static inline bool cb_init(circ_buf_t *cb, size_t capacity, size_t sz)
{
  cb->buffer = malloc(capacity * sz);
  if(cb->buffer == NULL)
    return false; // handle error
  cb->buffer_end = (int8_t *)cb->buffer + capacity * sz;
  cb->capacity = capacity;
  cb->count = 0;
  cb->sz = sz;
  cb->head = cb->buffer;
  cb->tail = cb->buffer;
  return true;
}

static inline void cb_free(circ_buf_t *cb)
{
  free(cb->buffer);
  cb->buffer = NULL;
  // clear out other fields too, just to be safe
  cb->buffer_end = 0;
  cb->capacity = 0;
  cb->count = 0;
  cb->sz = 0;
  cb->head = 0;
  cb->tail = 0;
}

static inline bool cb_has_room(circ_buf_t *cb)
{
  if(cb->count == cb->capacity)
    return false;
  return true;
}

static inline bool cb_push_back(circ_buf_t *cb, const void *item)
{
  if(cb->count == cb->capacity)
    return false; // handle error
  memcpy(cb->head, item, cb->sz);
  cb->head = (int8_t *)cb->head + cb->sz;
  if(cb->head == cb->buffer_end)
    cb->head = cb->buffer;
  cb->count++;
  return true;
}

static inline bool cb_pop_front(circ_buf_t *cb, void *item)
{
  if(cb->count == 0)
    return false; // handle error
  memcpy(item, cb->tail, cb->sz);
  cb->tail = (int8_t *)cb->tail + cb->sz;
  if(cb->tail == cb->buffer_end)
    cb->tail = cb->buffer;
  cb->count--;
  return true;
}

static inline bool cb_pop_front_demux(circ_buf_t *cb, void *item_i, void *item_q)
{
  if(cb->count == 0)
    return false; // handle error
  for(unsigned int nn=0;nn < cb->sz/2;nn++)
  {
        ((int8_t *)item_i)[nn]=((int8_t *)cb->tail)[2*nn];
        ((int8_t *)item_q)[nn]=((int8_t *)cb->tail)[2*nn+1];
  }
  cb->tail = (int8_t *)cb->tail + cb->sz;
  if(cb->tail == cb->buffer_end)
    cb->tail = cb->buffer;
  cb->count--;
  return true;
}


fl2k_sink_c_sptr make_fl2k_sink_c(const std::string &args)
{
  return gnuradio::get_initial_sptr(new fl2k_sink_c(args));
}

fl2k_sink_c::fl2k_sink_c (const std::string &args)
  : gr::sync_block ("fl2k_sink_c",
        gr::io_signature::make(MIN_IN, MAX_IN, sizeof (gr_complex)),
        gr::io_signature::make(MIN_OUT, MAX_OUT, sizeof (gr_complex))),
    _dev(NULL),
    _buf(NULL)
{
  _devnum = 0;
  _freq = 0;
  _buf_num = BUF_NUM;

  dict_t dict = params_to_dict(args);

  if (dict.count("fl2k"))    
    _devnum = boost::lexical_cast< unsigned int >( dict["fl2k"] );

  if (dict.count("freq"))
    _freq = boost::lexical_cast< double >( dict["freq"] );

  if (_freq < 0)
    throw std::runtime_error("Parameter 'freq' may not be negative.");

  _dev = NULL;
  fl2k_open( &_dev, 0);
  if (NULL == _dev) {
        throw std::runtime_error("Failed to open FL2K device" );
  }

  if ( BUF_NUM != _buf_num ) {
    std::cerr << "Using " << _buf_num << " buffers of size " << BUF_LEN << "."
              << std::endl;
  }

  _buf = (int8_t *) malloc( BUF_LEN );
  _rbuf = (int8_t *) malloc( FL2K_BUF_LEN );
  _gbuf = (int8_t *) malloc( FL2K_BUF_LEN );

  cb_init( &_cbuf, _buf_num, BUF_LEN );

  set_sample_rate( get_sample_rates().start() );

  fl2k_start_tx(_dev, _fl2k_callback, (void *)this, 0);
}

fl2k_sink_c::~fl2k_sink_c()
{
    if (_dev) {
          fl2k_close(_dev);
          _dev = NULL;
    }

    free(_buf);
    free(_rbuf);
    free(_gbuf);
    _buf = NULL;
    _rbuf = NULL;
    _gbuf = NULL;

    cb_free( &_cbuf );

}

void fl2k_sink_c::_fl2k_callback(fl2k_data_info_t *data_info)
{
  fl2k_sink_c *obj = (fl2k_sink_c *)data_info->ctx;
  data_info->sampletype_signed = 1;
  data_info->r_buf=(char *)obj->_rbuf;
  data_info->g_buf=(char *)obj->_gbuf;
  obj->fl2k_callback((unsigned char*)data_info->r_buf, (unsigned char*)data_info->g_buf, data_info->len);
  return;
}

int fl2k_sink_c::fl2k_callback(unsigned char *ibuffer, unsigned char *qbuffer, uint32_t length)
{
  {
    boost::mutex::scoped_lock lock( _buf_mutex );

    if ( ! cb_pop_front_demux( &_cbuf, ibuffer, qbuffer ) ) {
      memset(ibuffer, 0, length);
      memset(qbuffer, 0, length);
      std::cerr << "U" << std::flush;
    } else {
//      std::cerr << "-" << std::flush;

      _buf_cond.notify_one();
    }
  }
  return 0; // TODO: return -1 on error/stop
}

std::string fl2k_sink_c::name()
{
  return "Fresco Logic FL2000 sink";
}

#ifdef USE_AVX
void conv_avx(const float* inbuf, int8_t* outbuf,const unsigned int count)
{
  __m256 mulme = _mm256_set_ps(127.0f, 127.0f, 127.0f, 127.0f, 127.0f, 127.0f, 127.0f, 127.0f);
  for(unsigned int i=0; i<count;i++){

  __m256i itmp3 = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(&inbuf[i*16+0]), mulme));
  __m256i itmp4 = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(&inbuf[i*16+8]), mulme));

  __m128i a1 = _mm256_extractf128_si256(itmp3, 1);
  __m128i a0 = _mm256_castsi256_si128(itmp3);
  __m128i a3 = _mm256_extractf128_si256(itmp4, 1);
  __m128i a2 = _mm256_castsi256_si128(itmp4);

  __m128i outshorts1 = _mm_packs_epi32(a0, a1);
  __m128i outshorts2 = _mm_packs_epi32(a2, a3);

  __m128i outbytes = _mm_packs_epi16(outshorts1, outshorts2);

  _mm_storeu_si128 ((__m128i*)&outbuf[i*16], outbytes);
  }
}

#elif USE_SSE2
void conv_sse2(const float* inbuf, int8_t* outbuf,const unsigned int count)
{
  const register __m128 mulme = _mm_set_ps( 127.0f, 127.0f, 127.0f, 127.0f );
  __m128 itmp1,itmp2,itmp3,itmp4;
  __m128i otmp1,otmp2,otmp3,otmp4;

  __m128i outshorts1,outshorts2;
  __m128i outbytes;

  for(unsigned int i=0; i<count;i++){

  itmp1 = _mm_mul_ps(_mm_loadu_ps(&inbuf[i*16+0]), mulme);
  itmp2 = _mm_mul_ps(_mm_loadu_ps(&inbuf[i*16+4]), mulme);
  itmp3 = _mm_mul_ps(_mm_loadu_ps(&inbuf[i*16+8]), mulme);
  itmp4 = _mm_mul_ps(_mm_loadu_ps(&inbuf[i*16+12]), mulme);

  otmp1 = _mm_cvtps_epi32(itmp1);
  otmp2 = _mm_cvtps_epi32(itmp2);
  otmp3 = _mm_cvtps_epi32(itmp3);
  otmp4 = _mm_cvtps_epi32(itmp4);

  outshorts1 = _mm_packs_epi32(otmp1, otmp2);
  outshorts2 = _mm_packs_epi32(otmp3, otmp4);

  outbytes = _mm_packs_epi16(outshorts1, outshorts2);

  _mm_storeu_si128 ((__m128i*)&outbuf[i*16], outbytes);
  }
}
#endif

void conv_default(float* inbuf, int8_t* outbuf,const unsigned int count)
{
  for(unsigned int i=0; i<count;i++){
    outbuf[i]= inbuf[i]*127;
  }
}

int fl2k_sink_c::work( int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items )
{
  const gr_complex *in = (const gr_complex *) input_items[0];

  {
    boost::mutex::scoped_lock lock( _buf_mutex );

    while ( ! cb_has_room(&_cbuf) )
      _buf_cond.wait( lock );
  }

  int8_t *buf = _buf + _buf_used;
  unsigned int prev_buf_used = _buf_used;

  unsigned int remaining = (BUF_LEN-_buf_used)/2; //complex

  unsigned int count = std::min((unsigned int)noutput_items,remaining);
  unsigned int sse_rem = count/8; // 8 complex = 16f==512bit for avx
  unsigned int nosse_rem = count%8; // remainder

#ifdef USE_AVX
  conv_avx((float*)in, buf, sse_rem);
  conv_default((float*)(in+sse_rem*8), buf+(sse_rem*8*2), nosse_rem*2);
#elif USE_SSE2
  conv_sse2((float*)in, buf, sse_rem);
  conv_default((float*)(in+sse_rem*8), buf+(sse_rem*8*2), nosse_rem*2);
#else
  conv_default((float*)in, buf, count*2);
#endif

  _buf_used += (sse_rem*8+nosse_rem)*2;
  int items_consumed = sse_rem*8+nosse_rem;

  if((unsigned int)noutput_items >= remaining) {
    {
      boost::mutex::scoped_lock lock( _buf_mutex );

      if ( ! cb_push_back( &_cbuf, _buf ) ) {
        _buf_used = prev_buf_used;
        items_consumed = 0;
        std::cerr << "O" << std::flush;
      } else {
//        std::cerr << "+" << std::flush;
        _buf_used = 0;
      }
    }
  }

  // Tell runtime system how many input items we consumed on
  // each input stream.
  consume_each(items_consumed);

  // Tell runtime system how many output items we produced.
  return 0;
}


void fl2k_sink_c::_fl2k_wait(fl2k_sink_c *obj)
{
  obj->fl2k_wait();
}

void fl2k_sink_c::fl2k_wait()
{
}

bool fl2k_sink_c::start()
{
  if ( ! _dev )
    return false;

  _buf_used = 0;
  return true;
}

bool fl2k_sink_c::stop()
{
  if ( ! _dev )
    return false;
  return true;
}


std::vector<std::string> fl2k_sink_c::get_devices( bool fake )
{
  std::vector<std::string> devices;

  fl2k_dev_t *dev = NULL;
  unsigned int devnum = 0;
  fl2k_open(&dev, devnum);
  if (NULL != dev) {
      std::string args = "fl2k=devnum";
      args += ",freq=dontcare";
      args += ",label='FL2K output'";
      devices.push_back( args );
    fl2k_close(dev);
  }

  return devices;
}

size_t fl2k_sink_c::get_num_channels( void )
{
  return 1;
}

osmosdr::meta_range_t fl2k_sink_c::get_sample_rates( void )
{
  osmosdr::meta_range_t range;

  range += osmosdr::range_t(10e6, 160e6);

  return range;
}

double fl2k_sink_c::set_sample_rate( double rate )
{
    int ret;

    if (_dev) {
      ret = fl2k_set_sample_rate( _dev, rate );
      if (ret < 0)
            fprintf(stderr, "WARNING: Failed to set sample rate.\n");

      _rate = rate;
    }
    return get_sample_rate();
}

double fl2k_sink_c::get_sample_rate( void )
{
  return _rate;
}

osmosdr::freq_range_t fl2k_sink_c::get_freq_range( size_t chan )
{
  return osmosdr::freq_range_t(_freq, _freq);
}

double fl2k_sink_c::set_center_freq( double freq, size_t chan )
{
  return get_center_freq(chan);
}

double fl2k_sink_c::get_center_freq( size_t chan )
{
  return _freq;
}

double fl2k_sink_c::set_freq_corr( double ppm, size_t chan )
{
  return get_freq_corr( chan );
}

double fl2k_sink_c::get_freq_corr( size_t chan )
{
  return 0;
}

std::vector<std::string> fl2k_sink_c::get_gain_names( size_t chan )
{
  return std::vector< std::string >();
}

osmosdr::gain_range_t fl2k_sink_c::get_gain_range( size_t chan )
{
  return osmosdr::gain_range_t();
}

osmosdr::gain_range_t fl2k_sink_c::get_gain_range( const std::string & name, size_t chan )
{
  return get_gain_range( chan );
}

double fl2k_sink_c::set_gain( double gain, size_t chan )
{
  return get_gain(chan);
}

double fl2k_sink_c::set_gain( double gain, const std::string & name, size_t chan )
{
  return set_gain(chan);
}

double fl2k_sink_c::get_gain( size_t chan )
{
  return 0;
}

double fl2k_sink_c::get_gain( const std::string & name, size_t chan )
{
  return get_gain(chan);
}

std::vector< std::string > fl2k_sink_c::get_antennas( size_t chan )
{
  return std::vector< std::string >();
}

std::string fl2k_sink_c::set_antenna( const std::string & antenna, size_t chan )
{
  return get_antenna(chan);
}

std::string fl2k_sink_c::get_antenna( size_t chan )
{
  return "";
}
