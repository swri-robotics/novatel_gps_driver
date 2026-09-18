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


#include <sstream>

#include <novatel_gps_driver/parsers/rawimux.h>

#include <novatel_gps_driver/parsers/header.h>

namespace novatel_gps_driver
{
  const std::string RawImuxParser::MESSAGE_NAME = "RAWIMUX";

  uint32_t RawImuxParser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string RawImuxParser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  RawImuxParser::MessageType RawImuxParser::ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected RAWIMUX message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
    auto ros_msg = std::make_unique<novatel_gps_msgs::msg::NovatelRawImu>();
    HeaderParser header_parser;
    if (bin_msg.header_.header_length_ == HeaderParser::BINARY_SHORT_HEADER_LENGTH)
    {
      ros_msg->novatel_msg_header = header_parser.ParseShortBinary(bin_msg);
      ros_msg->novatel_msg_header.message_name = "RAWIMUSX";
    }
    else
    {
      ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
      ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;
    }

    ros_msg->imu_info = bin_msg.data_[0];
    ros_msg->imu_type = bin_msg.data_[1];
    ros_msg->gps_week_num = ParseUInt16(&bin_msg.data_[2]);
    ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
    std::copy(&bin_msg.data_[12], &bin_msg.data_[16], ros_msg->imu_status.begin());
    ros_msg->z_acceleration = ParseInt32(&bin_msg.data_[16]);
    ros_msg->negated_y_acceleration = ParseInt32(&bin_msg.data_[20]);
    ros_msg->x_acceleration = ParseInt32(&bin_msg.data_[24]);
    ros_msg->z_rotation = ParseInt32(&bin_msg.data_[28]);
    ros_msg->negated_y_rotation = ParseInt32(&bin_msg.data_[32]);
    ros_msg->x_rotation = ParseInt32(&bin_msg.data_[36]);

    return ros_msg;
  }

  RawImuxParser::MessageType RawImuxParser::ParseAscii(const NovatelSentence& sentence) noexcept(false)
  {
    auto msg = std::make_unique<novatel_gps_msgs::msg::NovatelRawImu>();
    HeaderParser h_parser;
    msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of RAWIMUX message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    valid = valid && ParseUInt8(sentence.body[0], msg->imu_info, 16);
    valid = valid && ParseUInt8(sentence.body[1], msg->imu_type);
    valid = valid && ParseUInt32(sentence.body[2], msg->gps_week_num);
    valid = valid && ParseDouble(sentence.body[3], msg->gps_seconds);
    // The status is printed as one hex pair per byte, in the order they're sent.
    const std::string& status = sentence.body[4];
    valid = valid && status.size() == 2 * msg->imu_status.size();
    for (size_t i = 0; valid && i < msg->imu_status.size(); i++)
    {
      valid = ParseUInt8(status.substr(2 * i, 2), msg->imu_status[i], 16);
    }
    valid = valid && ParseInt32(sentence.body[5], msg->z_acceleration);
    valid = valid && ParseInt32(sentence.body[6], msg->negated_y_acceleration);
    valid = valid && ParseInt32(sentence.body[7], msg->x_acceleration);
    valid = valid && ParseInt32(sentence.body[8], msg->z_rotation);
    valid = valid && ParseInt32(sentence.body[9], msg->negated_y_rotation);
    valid = valid && ParseInt32(sentence.body[10], msg->x_rotation);

    if (!valid)
    {
      throw ParseException("Invalid field in RAWIMUX message");
    }

    return msg;
  }
}
