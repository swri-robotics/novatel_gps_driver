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

#ifndef NOVATEL_GPS_DRIVER_BINARY_MESSAGE_H
#define NOVATEL_GPS_DRIVER_BINARY_MESSAGE_H

#include <novatel_gps_driver/binary_header.h>

#include <vector>

namespace novatel_gps_driver
{
  /**
   * Contains the header, raw data bytes, and CRC of a binary NovAtel message.
   */
  struct BinaryMessage
  {
    BinaryHeader header_;
    std::vector<uint8_t> data_;
    uint32_t crc_;
  };
}

#endif //NOVATEL_GPS_DRIVER_BINARY_MESSAGE_H
