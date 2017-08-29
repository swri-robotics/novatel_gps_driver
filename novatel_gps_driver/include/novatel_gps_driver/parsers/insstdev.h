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

#ifndef NOVATEL_GPS_DRIVER_INSSTDEV_H
#define NOVATEL_GPS_DRIVER_INSSTDEV_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Insstdev.h>

namespace novatel_gps_driver
{
  class InsstdevParser : public MessageParser<novatel_gps_msgs::InsstdevPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::InsstdevPtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::InsstdevPtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr uint32_t MESSAGE_ID = 2051;
    static const std::string MESSAGE_NAME;
    static constexpr size_t BINARY_LENGTH = 52;
    static constexpr size_t ASCII_FIELDS = 14;
  };
}

#endif //NOVATEL_GPS_DRIVER_INSSTDEV_H
