// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <novatel_gps_driver/parsers/time.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::TimeParser::MESSAGE_NAME = "TIME";

uint32_t novatel_gps_driver::TimeParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::TimeParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::TimePtr novatel_gps_driver::TimeParser::ParseBinary(const novatel_gps_driver::BinaryMessage& msg) throw(ParseException)
{
  if (msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected time message size: " << msg.data_.size();
    throw ParseException(error.str());
  }

  novatel_gps_msgs::TimePtr ros_msg = boost::make_shared<novatel_gps_msgs::Time>();

  uint32_t clock_status = ParseUInt32(&msg.data_[0]);
  switch (clock_status)
  {
    case 0:
      ros_msg->clock_status = "VALID";
      break;
    case 1:
      ros_msg->clock_status = "CONVERGING";
      break;
    case 2:
      ros_msg->clock_status = "ITERATING";
      break;
    case 3:
      ros_msg->clock_status = "INVALID";
      break;
    default:
    {
      std::stringstream error;
      error << "Unexpected clock status: " << clock_status;
      throw ParseException(error.str());
    }
  }
  ros_msg->offset = ParseDouble(&msg.data_[4]);
  ros_msg->offset_std = ParseDouble(&msg.data_[12]);
  ros_msg->utc_offset = ParseDouble(&msg.data_[20]);
  ros_msg->utc_year = ParseUInt32(&msg.data_[28]);
  ros_msg->utc_month = msg.data_[32];
  ros_msg->utc_day = msg.data_[33];
  ros_msg->utc_hour = msg.data_[34];
  ros_msg->utc_minute = msg.data_[35];
  ros_msg->utc_millisecond = ParseUInt32(&msg.data_[36]);
  uint32_t utc_status = ParseUInt32(&msg.data_[40]);
  switch (utc_status)
  {
    case 0:
      ros_msg->utc_status = "Invalid";
      break;
    case 1:
      ros_msg->utc_status = "Valid";
      break;
    case 2:
      ros_msg->utc_status = "Warning";
      break;
    default:
    {
      std::stringstream error;
      error << "Unexpected UTC status: " << utc_status;
      throw ParseException(error.str());
    }
  }

  return ros_msg;
}

novatel_gps_msgs::TimePtr
novatel_gps_driver::TimeParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException)
{
  novatel_gps_msgs::TimePtr msg = boost::make_shared<novatel_gps_msgs::Time>();
  if (sentence.body.size() != ASCII_FIELD)
  {
    std::stringstream error;
    error << "Unexpected number of fields in TIME log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  bool valid = true;
  msg->clock_status = sentence.body[0];
  valid &= ParseDouble(sentence.body[1], msg->offset);
  valid &= ParseDouble(sentence.body[2], msg->offset_std);
  valid &= ParseDouble(sentence.body[3], msg->utc_offset);
  valid &= ParseUInt32(sentence.body[4], msg->utc_year, 10);
  valid &= ParseUInt8(sentence.body[5], msg->utc_month, 10);
  valid &= ParseUInt8(sentence.body[6], msg->utc_day, 10);
  valid &= ParseUInt8(sentence.body[7], msg->utc_hour, 10);
  valid &= ParseUInt8(sentence.body[8], msg->utc_minute, 10);
  valid &= ParseUInt32(sentence.body[9], msg->utc_millisecond, 10);
  msg->utc_status = sentence.body[10];

  if (!valid)
  {
    throw ParseException("Error parsing TIME log.");
  }

  return msg;
}
