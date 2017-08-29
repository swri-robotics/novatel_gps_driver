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

#ifndef NOVATEL_GPS_DRIVER_INSCOV_H
#define NOVATEL_GPS_DRIVER_INSCOV_H

#include <novatel_gps_msgs/Inscov.h>
#include <novatel_gps_driver/parsers/message_parser.h>

namespace novatel_gps_driver
{
  class InscovParser : public MessageParser<novatel_gps_msgs::InscovPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::InscovPtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::InscovPtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr uint32_t MESSAGE_ID = 264;
    static const std::string MESSAGE_NAME;
    static constexpr size_t BINARY_LENGTH = 228;
    static constexpr size_t ASCII_FIELDS = 29;
  };
}

#endif //NOVATEL_GPS_DRIVER_INSCOV_H
