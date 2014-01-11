/* -*- c++ -*- */
/*
 * Copyright 2013 Nuand LLC
 * Copyright 2013 Dimitri Stolnikov <horiz0n@gmx.net>
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

/*
 * config.h is generated by configure.  It contains the results
 * of probing for features, options etc.  It should be the first
 * file included in your .cc file.
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include "bladerf_common.h"

#define NUM_BUFFERS 32
#define NUM_SAMPLES_PER_BUFFER (4 * 1024)

using namespace boost::assign;

boost::mutex bladerf_common::_devs_mutex;
std::list<boost::weak_ptr<struct bladerf> > bladerf_common::_devs;

bladerf_common::bladerf_common() : _is_running(false) {}
bladerf_common::~bladerf_common() {}

bladerf_sptr bladerf_common:: get_cached_device(struct bladerf_devinfo devinfo)
{
  /* Lock to _devs must be aquired by caller */
  BOOST_FOREACH( boost::weak_ptr<struct bladerf> dev, _devs )
  {
    struct bladerf_devinfo other_devinfo;

    int rv = bladerf_get_devinfo(bladerf_sptr(dev).get(), &other_devinfo);
    if (rv < 0)
      throw std::runtime_error(std::string(__FUNCTION__) + " " +
                               "Failed to get devinfo for cached device.");

    if (bladerf_devinfo_matches(&devinfo, &other_devinfo)) {
      return bladerf_sptr(dev);
    }
  }

  return bladerf_sptr();
}

void bladerf_common::close(void* dev)
{
  boost::unique_lock<boost::mutex> lock(_devs_mutex);

  std::list<boost::weak_ptr<struct bladerf> >::iterator it;
  for (it = _devs.begin(); it != _devs.end(); ++it)
    if ( (*it).expired() == 0 )
      _devs.erase(it);

  bladerf_close((struct bladerf *)dev);
}

bladerf_sptr bladerf_common::open(const std::string &device_name)
{
  int rv;
  struct bladerf *raw_dev;
  struct bladerf_devinfo devinfo;

  boost::unique_lock<boost::mutex> lock(_devs_mutex);

  rv = bladerf_get_devinfo_from_str(device_name.c_str(), &devinfo);
  if (rv < 0)
    throw std::runtime_error(std::string(__FUNCTION__) + " " +
                             "Failed to get devinfo for '" + device_name + "'");

  bladerf_sptr cached_dev = get_cached_device(devinfo);

  if (cached_dev)
    return cached_dev;

  rv = bladerf_open_with_devinfo(&raw_dev, &devinfo);
  if (rv < 0)
    throw std::runtime_error(std::string(__FUNCTION__) + " " +
                             "Failed to open device for '" + device_name + "'");

  bladerf_sptr dev = bladerf_sptr(raw_dev, bladerf_common::close);

  _devs.push_back(boost::weak_ptr<struct bladerf>(dev));

  return dev;
}

void bladerf_common::init(dict_t &dict, const char *type)
{
  int ret;
  unsigned int device_number = 0;
  std::string device_name;
  struct bladerf_version ver;
  char serial[BLADERF_SERIAL_LENGTH];

  _pfx = std::string("[bladeRF ") + std::string(type) + std::string("] ");

  if (dict.count("bladerf"))
  {
    std::string value = dict["bladerf"];
    if ( value.length() )
    {
      try {
        device_number = boost::lexical_cast< unsigned int >( value );
      } catch ( std::exception &ex ) {
        throw std::runtime_error( _pfx + "Failed to use '" + value +
                                  "' as device number: " + ex.what());
      }
    }
  }

  device_name = boost::str(boost::format( "libusb:instance=%d" ) % device_number);

  try {
    _dev = open(device_name);
  } catch(...) {
    throw std::runtime_error( _pfx + "Failed to open bladeRF device " +
                              device_name );
  }

  /* Load an FPGA */
  if ( dict.count("fpga") )
  {

    if ( dict.count("fpga-reload") == 0 &&
         bladerf_is_fpga_configured( _dev.get() ) == 1 ) {

      std::cerr << _pfx << "FPGA is already loaded. Set fpga-reload=1 "
                << "to force a reload." << std::endl;

    } else {
      std::string fpga = dict["fpga"];

      std::cerr << _pfx << "Loading FPGA bitstream " << fpga << "..." << std::endl;
      ret = bladerf_load_fpga( _dev.get(), fpga.c_str() );
      if ( ret != 0 )
        std::cerr << _pfx << "bladerf_load_fpga has failed with " << ret << std::endl;
      else
        std::cerr << _pfx << "The FPGA bitstream has been successfully loaded." << std::endl;
    }
  }

  if ( bladerf_is_fpga_configured( _dev.get() ) != 1 )
  {
    std::ostringstream oss;
    oss << _pfx << "The FPGA is not configured! "
        << "Provide device argument fpga=/path/to/the/bitstream.rbf to load it.";

    throw std::runtime_error( oss.str() );
  }


  /* Show some info about the device we've opened */
  std::cerr << _pfx << "Using nuand LLC bladeRF #" << device_number;

  if ( bladerf_get_serial( _dev.get(), serial ) == 0 )
  {
    std::string strser(serial);

    if ( strser.length() == 32 )
      strser.replace( 4, 24, "..." );

    std::cerr << " SN " << strser;
  }

  if ( bladerf_fw_version( _dev.get(), &ver ) == 0 )
    std::cerr << " FW v" << ver.major << "." << ver.minor << "." << ver.patch;

  if ( bladerf_fpga_version( _dev.get(), &ver ) == 0 )
    std::cerr << " FPGA v" << ver.major << "." << ver.minor << "." << ver.patch;

  std::cerr << std::endl;

  /* Initialize buffer and sample configuration */
  _num_buffers = 0;
  if (dict.count("buffers")) {
    _num_buffers = boost::lexical_cast< size_t >( dict["buffers"] );
  }

  _samples_per_buffer = 0;
  if (dict.count("buflen")) {
    _samples_per_buffer = boost::lexical_cast< size_t >( dict["buflen"] );
  }

  _num_transfers = 0;
  if (dict.count("transfers")) {
    _num_transfers = boost::lexical_cast< size_t >( dict["transfers"] );
  }

  /* Require value to be >= 2 so we can ensure we have twice as many
   * buffers as transfers */
  if (_num_buffers <= 1) {
    _num_buffers = NUM_BUFFERS;
  }

  if (0 == _samples_per_buffer) {
    _samples_per_buffer = NUM_SAMPLES_PER_BUFFER;
  } else {
    if (_samples_per_buffer < 1024 || _samples_per_buffer % 1024 != 0) {

      /* 0 likely implies the user did not specify this, so don't warn */
      if (_samples_per_buffer != 0 ) {
        std::cerr << _pfx << "Invalid \"buflen\" value. "
                  << "A multiple of 1024 is required. Defaulting to "
                  << NUM_SAMPLES_PER_BUFFER << std::endl;
      }

      _samples_per_buffer = NUM_SAMPLES_PER_BUFFER;
    }
  }



  if (_num_transfers == 0 || _num_transfers > (_num_buffers / 2)) {
      _num_transfers = _num_buffers / 2;
  }
}

osmosdr::freq_range_t bladerf_common::freq_range()
{
  /* assuming the same for RX & TX */
  return osmosdr::freq_range_t( 300e6, 3.8e9 );
}

osmosdr::meta_range_t bladerf_common::sample_rates()
{
  osmosdr::meta_range_t sample_rates;

  /* assuming the same for RX & TX */
  sample_rates += osmosdr::range_t( 160e3, 200e3, 40e3 );
  sample_rates += osmosdr::range_t( 300e3, 900e3, 100e3 );
  sample_rates += osmosdr::range_t( 1e6, 40e6, 1e6 );

  return sample_rates;
}

osmosdr::freq_range_t bladerf_common::filter_bandwidths()
{
  /* the same for RX & TX according to the datasheet */
  osmosdr::freq_range_t bandwidths;

  std::vector<double> half_bandwidths; /* in MHz */
  half_bandwidths += \
      0.75, 0.875, 1.25, 1.375, 1.5, 1.92, 2.5,
      2.75, 3, 3.5, 4.375, 5, 6, 7, 10, 14;

  BOOST_FOREACH( double half_bw, half_bandwidths )
    bandwidths += osmosdr::range_t( half_bw * 2e6 );

  return bandwidths;
}

std::vector< std::string > bladerf_common::devices()
{
  struct bladerf_devinfo *devices;
  ssize_t n_devices;
  std::vector< std::string > ret;

  n_devices = bladerf_get_device_list(&devices);

  if (n_devices > 0)
  {
    for (ssize_t i = 0; i < n_devices; i++)
    {
      std::stringstream s;
      std::string serial(devices[i].serial);

      s << "bladerf=" << devices[i].instance << ","
        << "label='nuand bladeRF";

      if ( serial.length() == 32 )
        serial.replace( 4, 24, "..." );

      if ( serial.length() )
        s << " SN " << serial;

      s << "'";

      ret.push_back(s.str());
    }

    bladerf_free_device_list(devices);
  }

  return ret;
}

bool bladerf_common::is_running()
{
  boost::shared_lock<boost::shared_mutex> lock(_state_lock);

  return _is_running;
}

void bladerf_common::set_running( bool is_running )
{
  boost::unique_lock<boost::shared_mutex> lock(_state_lock);

  _is_running = is_running;
}