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

#ifndef NOVATEL_GPS_DRIVER_BESTVEL_H_H
#define NOVATEL_GPS_DRIVER_BESTVEL_H_H

#include <novatel_gps_driver/parsers/message_parser.h>

#include <novatel_gps_msgs/NovatelVelocity.h>

namespace novatel_gps_driver
{
  class BestvelParser : public MessageParser<novatel_gps_msgs::NovatelVelocityPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::NovatelVelocityPtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::NovatelVelocityPtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr uint16_t MESSAGE_ID = 99;
    static constexpr size_t ASCII_LENGTH = 8;
    static constexpr size_t BINARY_LENGTH = 44;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_BESTVEL_H_H
