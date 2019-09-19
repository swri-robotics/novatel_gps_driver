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


#include <novatel_gps_driver/parsers/bestvel.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::BestvelParser::MESSAGE_NAME = "BESTVEL";

uint32_t novatel_gps_driver::BestvelParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::BestvelParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::NovatelVelocityPtr novatel_gps_driver::BestvelParser::ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected velocity message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelVelocityPtr ros_msg = boost::make_shared<novatel_gps_msgs::NovatelVelocity>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

  uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
  if (solution_status > MAX_SOLUTION_STATUS)
  {
    std::stringstream error;
    error << "Unknown solution status: " << solution_status;
    throw ParseException(error.str());
  }
  ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
  uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
  if (pos_type > MAX_POSITION_TYPE)
  {
    std::stringstream error;
    error << "Unknown position type: " << pos_type;
    throw ParseException(error.str());
  }
  ros_msg->velocity_type = POSITION_TYPES[pos_type];
  ros_msg->latency = ParseFloat(&bin_msg.data_[8]);
  ros_msg->age = ParseFloat(&bin_msg.data_[12]);
  ros_msg->horizontal_speed = ParseDouble(&bin_msg.data_[16]);
  ros_msg->track_ground = ParseDouble(&bin_msg.data_[24]);
  ros_msg->vertical_speed = ParseDouble(&bin_msg.data_[32]);

  return ros_msg;
}

novatel_gps_msgs::NovatelVelocityPtr novatel_gps_driver::BestvelParser::ParseAscii(const NovatelSentence& sentence) noexcept(false)
{
  novatel_gps_msgs::NovatelVelocityPtr msg = boost::make_shared<novatel_gps_msgs::NovatelVelocity>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  if (sentence.body.size() != ASCII_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected number of BESTVEL message fields: " << sentence.body.size();
    throw ParseException(error.str());
  }
  bool valid = true;
  msg->solution_status = sentence.body[0];
  msg->velocity_type = sentence.body[1];
  valid = valid && ParseFloat(sentence.body[2], msg->latency);
  valid = valid && ParseFloat(sentence.body[3], msg->age);
  valid = valid && ParseDouble(sentence.body[4], msg->horizontal_speed);
  valid = valid && ParseDouble(sentence.body[5], msg->track_ground);
  valid = valid && ParseDouble(sentence.body[6], msg->vertical_speed);

  if (!valid)
  {
    throw ParseException("Invalid field in BESTVEL message");
  }

  return msg;
}
