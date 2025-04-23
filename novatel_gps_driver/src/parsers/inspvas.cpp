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

#include <novatel_gps_driver/parsers/inspvas.h>
#include <novatel_gps_driver/parsers/header.h>

const std::string novatel_gps_driver::InspvasParser::MESSAGE_NAME = "INSPVAS";

uint32_t novatel_gps_driver::InspvasParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::InspvasParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::InspvasParser::MessageType
novatel_gps_driver::InspvasParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected inspva message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  auto ros_msg = std::make_shared<novatel_gps_msgs::msg::Inspva>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseShortBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();

  ros_msg->week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->latitude = ParseDouble(&bin_msg.data_[12]);
  ros_msg->longitude = ParseDouble(&bin_msg.data_[20]);
  ros_msg->height = ParseDouble(&bin_msg.data_[28]);
  ros_msg->north_velocity = ParseDouble(&bin_msg.data_[36]);
  ros_msg->east_velocity = ParseDouble(&bin_msg.data_[44]);
  ros_msg->up_velocity = ParseDouble(&bin_msg.data_[52]);
  ros_msg->roll = ParseDouble(&bin_msg.data_[60]);
  ros_msg->pitch = ParseDouble(&bin_msg.data_[68]);
  ros_msg->azimuth = ParseDouble(&bin_msg.data_[76]);
  uint32_t status = ParseUInt32(&bin_msg.data_[84]);

  switch (status)
  {
    case 0:
      ros_msg->status = "INS_INACTIVE";
      break;
    case 1:
      ros_msg->status = "INS_ALIGNING";
      break;
    case 2:
      ros_msg->status = "INS_HIGH_VARIANCE";
      break;
    case 3:
      ros_msg->status = "INS_SOLUTION_GOOD";
      break;
    case 6:
      ros_msg->status = "INS_SOLUTION_FREE";
      break;
    case 7:
      ros_msg->status = "INS_ALIGNMENT_COMPLETE";
      break;
    case 8:
      ros_msg->status = "DETERMINING_ORIENTATION";
      break;
    case 9:
      ros_msg->status = "WAITING_INITIALPOS";
      break;
    case 10:
      ros_msg->status = "WAITING_AZIMUTH";
      break;
    case 11:
      ros_msg->status = "INITIALIZING_BASES";
      break;
    case 12:
      ros_msg->status = "MOTION_DETECT";
      break;
    default:
    {
      std::stringstream error;
      error << "Unexpected inertial solution status: " << status;
      throw ParseException(error.str());
    }
  }

  return ros_msg;
}