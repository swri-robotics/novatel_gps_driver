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

#ifndef NOVATEL_GPS_DRIVER_RANGE_H
#define NOVATEL_GPS_DRIVER_RANGE_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Range.h>

namespace novatel_gps_driver
{
  class RangeParser : public MessageParser<novatel_gps_msgs::RangePtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::RangePtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::RangePtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr size_t BINARY_OBSERVATION_SIZE = 44;
    static constexpr uint16_t MESSAGE_ID = 43;
    static constexpr size_t ASCII_FIELDS = 10;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_RANGE_H
