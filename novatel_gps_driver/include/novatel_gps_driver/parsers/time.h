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
#ifndef NOVATEL_GPS_DRIVER_TIME_H
#define NOVATEL_GPS_DRIVER_TIME_H

#include <novatel_gps_driver/parsers/message_parser.h>
// novatel_gps_msgs generates the TIME log's message as NovatelTime on Rolling and
// newer and as Time on Lyrical and older; see the rosidl version check in
// novatel_gps_msgs/CMakeLists.txt for why. Key off the header that was actually
// generated rather than repeating that version check, so the driver stays correct
// even when built against a novatel_gps_msgs from a different distro.
#if __has_include(<novatel_gps_msgs/msg/novatel_time.hpp>)
#define NOVATEL_GPS_DRIVER_HAS_NOVATEL_TIME_MSG 1
#include <novatel_gps_msgs/msg/novatel_time.hpp>
#else
#define NOVATEL_GPS_DRIVER_HAS_NOVATEL_TIME_MSG 0
#include <novatel_gps_msgs/msg/time.hpp>
#endif

namespace novatel_gps_driver
{
#if NOVATEL_GPS_DRIVER_HAS_NOVATEL_TIME_MSG
  class TimeParser : public MessageParser<novatel_gps_msgs::msg::NovatelTime::UniquePtr>
#else
  class TimeParser : public MessageParser<novatel_gps_msgs::msg::Time::UniquePtr>
#endif
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    MessageType ParseBinary(const BinaryMessage& bin_msg) noexcept(false) override;

    MessageType ParseAscii(const NovatelSentence& sentence) noexcept(false) override;

    static constexpr size_t BINARY_LENGTH = 44;
    static constexpr uint16_t MESSAGE_ID = 101;
    static constexpr size_t ASCII_FIELD = 11;
    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_TIME_H
