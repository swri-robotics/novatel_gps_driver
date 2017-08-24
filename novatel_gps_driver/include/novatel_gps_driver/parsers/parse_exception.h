// *****************************************************************************
//
// Copyright (C) 2016 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#ifndef NOVATEL_GPS_DRIVER_PARSE_EXCEPTION_H
#define NOVATEL_GPS_DRIVER_PARSE_EXCEPTION_H

#include <exception>

namespace novatel_gps_driver
{
  /**
   * Thrown whenever a parser class has an unrecoverable issue  parsing a message.
   */
  class ParseException : public std::runtime_error
  {
  public:
    ParseException(const std::string& error) : std::runtime_error(error)
    {}
  };
}

#endif //NOVATEL_GPS_DRIVER_PARSE_EXCEPTION_H
