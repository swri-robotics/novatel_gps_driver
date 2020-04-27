// *****************************************************************************
//
// Copyright (C) 2019 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#ifndef NOVATEL_GPS_DRIVER_PSRDOP_2_H
#define NOVATEL_GPS_DRIVER_PSRDOP_2_H

#include <novatel_gps_msgs/msg/novatel_psrdop2.hpp>

#include "message_parser.h"

namespace novatel_gps_driver
{
  class Psrdop2Parser : public MessageParser<novatel_gps_msgs::msg::NovatelPsrdop2::SharedPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    MessageType ParseBinary(const BinaryMessage& bin_msg) override;

    MessageType ParseAscii(const NovatelSentence& sentence) override;

    std::string GetSystemName(uint32_t system_id);

    static constexpr uint16_t MESSAGE_ID = 1163;
    static constexpr size_t ASCII_BODY_FIELDS = 5;
    static constexpr size_t ASCII_SYSTEM_FIELDS = 2;
    static constexpr size_t BINARY_SYSTEM_LENGTH = 8;
    static constexpr size_t BINARY_BODY_LENGTH = 20;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_PSRDOP_2_H
