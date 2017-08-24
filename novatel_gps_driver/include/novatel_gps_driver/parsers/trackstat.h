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

#ifndef NOVATEL_GPS_DRIVER_TRACKSTAT_H_H
#define NOVATEL_GPS_DRIVER_TRACKSTAT_H_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Trackstat.h>

namespace novatel_gps_driver
{
  class TrackstatParser : public MessageParser<novatel_gps_msgs::TrackstatPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::TrackstatPtr ParseBinary(const BinaryMessage& bin_msg) throw(ParseException) override;

    novatel_gps_msgs::TrackstatPtr ParseAscii(const NovatelSentence& sentence) throw(ParseException) override;

    static constexpr uint16_t MESSAGE_ID = 83;
    static constexpr size_t ASCII_BODY_FIELDS = 4;
    static constexpr size_t ASCII_CHANNEL_FIELDS = 10;
    static constexpr size_t BINARY_CHANNEL_LENGTH = 40;
    static constexpr size_t BINARY_BODY_LENGTH = 16;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_TRACKSTAT_H_H
