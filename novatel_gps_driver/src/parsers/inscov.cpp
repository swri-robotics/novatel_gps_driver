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

#include <novatel_gps_driver/parsers/inscov.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::InscovParser::MESSAGE_NAME = "INSCOV";

uint32_t novatel_gps_driver::InscovParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::InscovParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::InscovPtr
novatel_gps_driver::InscovParser::ParseBinary(const BinaryMessage& bin_msg) throw(ParseException)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected inscov message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InscovPtr ros_msg = boost::make_shared<novatel_gps_msgs::Inscov>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();

  ros_msg->week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->seconds = ParseDouble(&bin_msg.data_[4]);
  int offset = 12;
  for (int i = 0; i < 9; i++, offset += 8)
  {
    ros_msg->position_covariance[i] = ParseDouble(&bin_msg.data_[offset]);
  }
  for (int i = 0; i < 9; i++, offset += 8)
  {
    ros_msg->attitude_covariance[i] = ParseDouble(&bin_msg.data_[offset]);
  }
  for (int i = 0; i < 9; i++, offset += 8)
  {
    ros_msg->velocity_covariance[i] = ParseDouble(&bin_msg.data_[offset]);
  }
  return ros_msg;
}

novatel_gps_msgs::InscovPtr
novatel_gps_driver::InscovParser::ParseAscii(const NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in INSCOV log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InscovPtr ros_msg = boost::make_shared<novatel_gps_msgs::Inscov>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], ros_msg->week);
  valid &= ParseDouble(sentence.body[1], ros_msg->seconds);

  int offset = 2;
  for (int i = 0; i < 9; i++, offset++)
  {
    valid &= ParseDouble(sentence.body[offset], ros_msg->position_covariance[i]);
  }
  for (int i = 0; i < 9; i++, offset++)
  {
    valid &= ParseDouble(sentence.body[offset], ros_msg->attitude_covariance[i]);
  }
  for (int i = 0; i < 9; i++, offset++)
  {
    valid &= ParseDouble(sentence.body[offset], ros_msg->velocity_covariance[i]);
  }

  if (!valid)
  {
    throw ParseException("Error parsing INSCOV log.");
  }

  return ros_msg;
}
