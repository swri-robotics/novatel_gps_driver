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

#include <novatel_gps_driver/parsers/inspvax.h>
#include <boost/make_shared.hpp>
#include <novatel_gps_driver/parsers/header.h>


const std::string novatel_gps_driver::InspvaxParser::MESSAGE_NAME = "INSPVAX";

uint32_t novatel_gps_driver::InspvaxParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::InspvaxParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::InspvaxPtr
novatel_gps_driver::InspvaxParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) throw(ParseException)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected inspvax message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InspvaxPtr ros_msg = boost::make_shared<novatel_gps_msgs::Inspvax>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();

  uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
  if (solution_status > MAX_SOLUTION_STATUS)
  {
    std::stringstream error;
    error << "Unknown solution status: " << solution_status;
    throw ParseException(error.str());
  }
  ros_msg->ins_status = SOLUTION_STATUSES[solution_status];
  uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
  if (pos_type > MAX_POSITION_TYPE)
  {
    std::stringstream error;
    error << "Unknown position type: " << pos_type;
    throw ParseException(error.str());
  }
  ros_msg->position_type = POSITION_TYPES[pos_type];  
  
  
  
  ros_msg->latitude = ParseDouble(&bin_msg.data_[8]);
  ros_msg->longitude = ParseDouble(&bin_msg.data_[16]);
  ros_msg->altitude = ParseDouble(&bin_msg.data_[24]);
  ros_msg->undulation = ParseFloat(&bin_msg.data_[32]);
  ros_msg->north_velocity = ParseDouble(&bin_msg.data_[36]);
  ros_msg->east_velocity = ParseDouble(&bin_msg.data_[44]);
  ros_msg->up_velocity = ParseDouble(&bin_msg.data_[52]);
  ros_msg->roll = ParseDouble(&bin_msg.data_[60]);
  ros_msg->pitch = ParseDouble(&bin_msg.data_[68]);
  ros_msg->azimuth = ParseDouble(&bin_msg.data_[76]);

  ros_msg->latitude_std = ParseFloat(&bin_msg.data_[84]);
  ros_msg->longitude_std = ParseFloat(&bin_msg.data_[88]);
  ros_msg->altitude_std = ParseFloat(&bin_msg.data_[92]);  

  ros_msg->north_velocity_std = ParseFloat(&bin_msg.data_[96]);
  ros_msg->east_velocity_std = ParseFloat(&bin_msg.data_[100]);
  ros_msg->up_velocity_std = ParseFloat(&bin_msg.data_[104]);

  ros_msg->roll_std = ParseFloat(&bin_msg.data_[108]);
  ros_msg->pitch_std = ParseFloat(&bin_msg.data_[112]);
  ros_msg->azimuth_std = ParseFloat(&bin_msg.data_[116]);
  GetExtendedSolutionStatusMessage(bin_msg.data_[120],
                                     ros_msg->extended_status);
  ros_msg->seconds_since_update = ParseUInt16(&bin_msg.data_[124]);

  return ros_msg;
}

novatel_gps_msgs::InspvaxPtr
novatel_gps_driver::InspvaxParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in INSPVA log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InspvaxPtr msg = boost::make_shared<novatel_gps_msgs::Inspvax>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  msg->ins_status = sentence.body[0];
  msg->position_type = sentence.body[1];
  valid &= ParseDouble(sentence.body[2], msg->latitude);
  valid &= ParseDouble(sentence.body[3], msg->longitude);
  valid &= ParseDouble(sentence.body[4], msg->altitude);
  valid &= ParseFloat(sentence.body[5], msg->undulation);
  valid &= ParseDouble(sentence.body[6], msg->north_velocity);
  valid &= ParseDouble(sentence.body[7], msg->east_velocity);
  valid &= ParseDouble(sentence.body[8], msg->up_velocity);
  valid &= ParseDouble(sentence.body[9], msg->roll);
  valid &= ParseDouble(sentence.body[10], msg->pitch);
  valid &= ParseDouble(sentence.body[11], msg->azimuth);
  valid &= ParseFloat(sentence.body[12], msg->latitude_std);
  valid &= ParseFloat(sentence.body[13], msg->longitude_std);
  valid &= ParseFloat(sentence.body[14], msg->altitude_std);
  valid &= ParseFloat(sentence.body[15], msg->north_velocity_std);
  valid &= ParseFloat(sentence.body[16], msg->east_velocity_std);
  valid &= ParseFloat(sentence.body[17], msg->up_velocity_std);
  valid &= ParseFloat(sentence.body[18], msg->roll_std);
  valid &= ParseFloat(sentence.body[19], msg->pitch_std);
  valid &= ParseFloat(sentence.body[20], msg->azimuth_std);

   // skip reserved field
    uint32_t extended_solution_status = 0;
    valid = valid && ParseUInt32(sentence.body[21], extended_solution_status, 16);
    GetExtendedSolutionStatusMessage(
        extended_solution_status, msg->extended_status);


  valid &= ParseUInt16(sentence.body[22], msg->seconds_since_update);
  

  if (!valid)
  {
    throw ParseException("Error parsing INSPVAX log.");
  }

  return msg;
}

