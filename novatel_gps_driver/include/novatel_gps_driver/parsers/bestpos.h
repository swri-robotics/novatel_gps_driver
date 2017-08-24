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

#ifndef NOVATEL_GPS_DRIVER_BESTPOS_H
#define NOVATEL_GPS_DRIVER_BESTPOS_H

#include <novatel_gps_msgs/NovatelPosition.h>

#include <novatel_gps_driver/parsers/parsing_utils.h>
#include <novatel_gps_driver/parsers/message_parser.h>

namespace novatel_gps_driver
{
  class BestposParser : public MessageParser<novatel_gps_msgs::NovatelPositionPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::NovatelPositionPtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::NovatelPositionPtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr uint16_t MESSAGE_ID = 42;
    static constexpr size_t BINARY_LENGTH = 72;
    static constexpr size_t ASCII_LENGTH = 21;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_BESTPOS_H
