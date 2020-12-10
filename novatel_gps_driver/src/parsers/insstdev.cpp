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

#include <novatel_gps_driver/parsers/insstdev.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::InsstdevParser::MESSAGE_NAME = "INSSTDEV";

uint32_t novatel_gps_driver::InsstdevParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::InsstdevParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::InsstdevPtr
novatel_gps_driver::InsstdevParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected INSSTDEV message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InsstdevPtr ros_msg = boost::make_shared<novatel_gps_msgs::Insstdev>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();
  ros_msg->latitude_dev = ParseFloat(&bin_msg.data_[0]);
  ros_msg->latitude_dev = ParseFloat(&bin_msg.data_[4]);
  ros_msg->height_dev = ParseFloat(&bin_msg.data_[8]);
  ros_msg->north_velocity_dev = ParseFloat(&bin_msg.data_[12]);
  ros_msg->east_velocity_dev = ParseFloat(&bin_msg.data_[16]);
  ros_msg->up_velocity_dev = ParseFloat(&bin_msg.data_[20]);
  ros_msg->roll_dev = ParseFloat(&bin_msg.data_[24]);
  ros_msg->pitch_dev = ParseFloat(&bin_msg.data_[28]);
  ros_msg->azimuth_dev = ParseFloat(&bin_msg.data_[32]);
  uint32_t status = ParseUInt32(&bin_msg.data_[36]);
  GetExtendedSolutionStatusMessage(status, ros_msg->extended_solution_status);
  ros_msg->time_since_update = ParseUInt16(&bin_msg.data_[40]);

  return ros_msg;
}

novatel_gps_msgs::InsstdevPtr
novatel_gps_driver::InsstdevParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in INSSTDEV log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InsstdevPtr msg = boost::make_shared<novatel_gps_msgs::Insstdev>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseFloat(sentence.body[0], msg->latitude_dev);
  valid &= ParseFloat(sentence.body[1], msg->longitude_dev);
  valid &= ParseFloat(sentence.body[2], msg->height_dev);
  valid &= ParseFloat(sentence.body[3], msg->north_velocity_dev);
  valid &= ParseFloat(sentence.body[4], msg->east_velocity_dev);
  valid &= ParseFloat(sentence.body[5], msg->up_velocity_dev);
  valid &= ParseFloat(sentence.body[6], msg->roll_dev);
  valid &= ParseFloat(sentence.body[7], msg->pitch_dev);
  valid &= ParseFloat(sentence.body[8], msg->azimuth_dev);
  uint32_t status;
  valid &= ParseUInt32(sentence.body[9], status);
  GetExtendedSolutionStatusMessage(status, msg->extended_solution_status);

  if (!valid)
  {
    throw ParseException("Error parsing INSSTDEV log.");
  }

  return msg;
}
