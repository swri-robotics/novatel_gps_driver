// *****************************************************************************
//
// Copyright (c) 2019, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef NOVATEL_GPS_DRIVER_BESTPOS_H
#define NOVATEL_GPS_DRIVER_BESTPOS_H

#include <novatel_gps_msgs/msg/novatel_position.hpp>

#include <novatel_gps_driver/parsers/parsing_utils.h>
#include <novatel_gps_driver/parsers/message_parser.h>

namespace novatel_gps_driver
{
  class BestposParser : public MessageParser<novatel_gps_msgs::msg::NovatelPosition::SharedPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    MessageType ParseBinary(const BinaryMessage& bin_msg) noexcept(false) override;

    MessageType ParseAscii(const NovatelSentence& sentence) noexcept(false) override;

    static constexpr uint16_t MESSAGE_ID = 42;
    static constexpr size_t BINARY_LENGTH = 72;
    static constexpr size_t ASCII_LENGTH = 21;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_BESTPOS_H
