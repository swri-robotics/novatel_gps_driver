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

#ifndef NOVATEL_GPS_DRIVER_GPGGA_H
#define NOVATEL_GPS_DRIVER_GPGGA_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Gpgga.h>

namespace novatel_gps_driver
{
  class GpggaParser : public MessageParser<novatel_gps_msgs::GpggaPtr>
  {
  public:
    GpggaParser(): MessageParser<novatel_gps_msgs::GpggaPtr>(),
                   was_last_gps_valid_(false)
    {}
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::GpggaPtr ParseAscii(const NmeaSentence& sentence) throw(ParseException) override;

    bool WasLastGpsValid() const;

    static const std::string MESSAGE_NAME;

  private:
    bool was_last_gps_valid_;
  };
}

#endif //NOVATEL_GPS_DRIVER_GPGGA_H
