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

#ifndef NOVATEL_GPS_DRIVER_HEADER_H
#define NOVATEL_GPS_DRIVER_HEADER_H

#include <novatel_gps_driver/parsers/parsing_utils.h>
#include <novatel_gps_driver/parsers/message_parser.h>

#include <novatel_gps_msgs/NovatelMessageHeader.h>

namespace novatel_gps_driver
{
  class HeaderParser : public MessageParser<novatel_gps_msgs::NovatelMessageHeader>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::NovatelMessageHeader ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::NovatelMessageHeader ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr uint32_t BINARY_HEADER_LENGTH = 28;
  };
}

#endif //NOVATEL_GPS_DRIVER_HEADER_H
