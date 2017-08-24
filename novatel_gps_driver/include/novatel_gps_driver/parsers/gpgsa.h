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

#ifndef NOVATEL_GPS_DRIVER_GPGSA_H
#define NOVATEL_GPS_DRIVER_GPGSA_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Gpgsa.h>

namespace novatel_gps_driver
{
  class GpgsaParser : public MessageParser<novatel_gps_msgs::GpgsaPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::GpgsaPtr ParseAscii(const NmeaSentence& sentence) throw(ParseException) override;

    static const std::string MESSAGE_NAME;
  };
};

#endif //NOVATEL_GPS_DRIVER_GPGSA_H
