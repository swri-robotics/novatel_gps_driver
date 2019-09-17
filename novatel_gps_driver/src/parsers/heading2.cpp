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

#include <novatel_gps_driver/parsers/heading2.h>

#include <novatel_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace novatel_gps_driver
{
  const std::string Heading2Parser::MESSAGE_NAME = "HEADING2";

  uint32_t Heading2Parser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string Heading2Parser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  novatel_gps_msgs::NovatelHeading2Ptr Heading2Parser::ParseBinary(const BinaryMessage& bin_msg) noexcept(false) 
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected HEADING2 message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
    novatel_gps_msgs::NovatelHeading2Ptr ros_msg =
        boost::make_shared<novatel_gps_msgs::NovatelHeading2>();
    HeaderParser header_parser;
    ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
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
    ros_msg->position_type = POSITION_TYPES[pos_type];

    ros_msg->baseline_length = ParseFloat(&bin_msg.data_[8]);

    ros_msg->heading = ParseFloat(&bin_msg.data_[12]);
    ros_msg->pitch = ParseFloat(&bin_msg.data_[16]);

    // Bytes 20-23 reserved 

    ros_msg->heading_sigma = ParseFloat(&bin_msg.data_[24]);
    ros_msg->pitch_sigma = ParseFloat(&bin_msg.data_[28]);

    ros_msg->rover_station_id.resize(4);
    std::copy(&bin_msg.data_[32], &bin_msg.data_[36], &ros_msg->rover_station_id[0]);

    ros_msg->master_station_id.resize(4);
    std::copy(&bin_msg.data_[36], &bin_msg.data_[40], &ros_msg->master_station_id[0]);

    ros_msg->num_satellites_tracked = bin_msg.data_[40];
    ros_msg->num_satellites_used_in_solution = bin_msg.data_[41];
    ros_msg->num_satellites_above_elevation_mask_angle = bin_msg.data_[42];
    ros_msg->num_satellites_above_elevation_mask_angle_l2 = bin_msg.data_[43];

    ros_msg->solution_source = SolutionSourceToMsgEnum(bin_msg.data_[44]);

    GetExtendedSolutionStatusMessage(bin_msg.data_[45],
                                  ros_msg->extended_solution_status);

    // Byte 46 is reserved

    GetSignalsUsed(bin_msg.data_[47], ros_msg->signal_mask);

    return ros_msg;
  }

  novatel_gps_msgs::NovatelHeading2Ptr Heading2Parser::ParseAscii(const NovatelSentence& sentence) noexcept(false)
  {
    novatel_gps_msgs::NovatelHeading2Ptr ros_msg =
        boost::make_shared<novatel_gps_msgs::NovatelHeading2>();
    HeaderParser h_parser;
    ros_msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of HEADING2 message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    ros_msg->solution_status = sentence.body[0];
    ros_msg->position_type = sentence.body[1];
    
    valid = valid && ParseFloat(sentence.body[2], ros_msg->baseline_length);

    valid = valid && ParseFloat(sentence.body[3], ros_msg->heading);
    valid = valid && ParseFloat(sentence.body[4], ros_msg->pitch);

    // Skip reserved field

    valid = valid && ParseFloat(sentence.body[6], ros_msg->heading_sigma);
    valid = valid && ParseFloat(sentence.body[7], ros_msg->pitch_sigma);

    ros_msg->rover_station_id = sentence.body[8];

    ros_msg->master_station_id = sentence.body[9];

    valid = valid && ParseUInt8(sentence.body[10], ros_msg->num_satellites_tracked);
    valid = valid && ParseUInt8(sentence.body[11], ros_msg->num_satellites_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[12], ros_msg->num_satellites_above_elevation_mask_angle);
    valid = valid && ParseUInt8(sentence.body[13], ros_msg->num_satellites_above_elevation_mask_angle_l2);

    uint32_t solution_source = 0;
    valid = valid && ParseUInt32(sentence.body[14], solution_source, 16);
    ros_msg->solution_source = SolutionSourceToMsgEnum((uint8_t)solution_source);

    uint32_t extended_solution_status = 0;
    valid = valid && ParseUInt32(sentence.body[15], extended_solution_status, 16);
    GetExtendedSolutionStatusMessage(
        extended_solution_status, ros_msg->extended_solution_status);

    // Skip reserved field

    uint32_t signal_mask = 0;
    valid = valid && ParseUInt32(sentence.body[17], signal_mask, 16);
    GetSignalsUsed(signal_mask, ros_msg->signal_mask);

    if (!valid)
    {
      throw ParseException("Invalid field in HEADING2 message");
    }

    return ros_msg;
  }

  uint8_t Heading2Parser::SolutionSourceToMsgEnum(uint8_t source_mask) noexcept(false)
  {
    uint8_t source_bits = (source_mask & 0x0Cu) >> 2u;
    switch (source_bits)
    {
      case 0:
        return novatel_gps_msgs::NovatelHeading2::SOURCE_PRIMARY_ANTENNA;
      case 1:
        return novatel_gps_msgs::NovatelHeading2::SOURCE_SECONDARY_ANTENNA;
      default:
        throw ParseException("HEADING2 Solution Source could not be parsed due to unknown source");
    }
  }
}
