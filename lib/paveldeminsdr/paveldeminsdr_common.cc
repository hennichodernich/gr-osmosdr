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

#include "paveldeminsdr_common.h"

void paveldeminsdr_send_commands( SOCKET socket, uint32_t *bufptr)
{
  std::stringstream message;

#if defined(_WIN32)
  int total = 10*sizeof(uint32_t);
  int size;
  size = ::send( socket, (char *)bufptr, 10*sizeof(uint32_t), 0 );
#else
  ssize_t total = 10*sizeof(uint32_t);
  ssize_t size;
  size = ::send( socket, bufptr, 10*sizeof(uint32_t), MSG_NOSIGNAL );
#endif

  if ( size != total )
  {
    message << "Sending command failed.";
    throw std::runtime_error( message.str() );
  }
}
