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

#include <novatel_gps_driver/parsers/bestxyz.h>

#include <novatel_gps_driver/parsers/header.h>

namespace novatel_gps_driver
{
  const std::string BestxyzParser::MESSAGE_NAME = "BESTXYZ";

  uint32_t BestxyzParser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string BestxyzParser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  BestxyzParser::MessageType BestxyzParser::ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected BESTXYZ message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
    auto ros_msg = std::make_unique<novatel_gps_msgs::msg::NovatelXYZ>();
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

    ros_msg->x = ParseDouble(&bin_msg.data_[8]);
    ros_msg->y = ParseDouble(&bin_msg.data_[16]);
    ros_msg->z = ParseDouble(&bin_msg.data_[24]);

    ros_msg->x_sigma = ParseFloat(&bin_msg.data_[32]);
    ros_msg->y_sigma = ParseFloat(&bin_msg.data_[36]);
    ros_msg->z_sigma = ParseFloat(&bin_msg.data_[40]);

    uint16_t vel_solution_status = ParseUInt16(&bin_msg.data_[44]);
    if (vel_solution_status > MAX_SOLUTION_STATUS)
    {
      std::stringstream error;
      error << "Unknown solution status: " << vel_solution_status;
      throw ParseException(error.str());
    }
    ros_msg->velocity_solution_status = SOLUTION_STATUSES[vel_solution_status];

    uint16_t vel_type = ParseUInt16(&bin_msg.data_[48]);
    if (vel_type > MAX_POSITION_TYPE) // Position types array includes velocity types
    {
      std::stringstream error;
      error << "Unknown position type: " << vel_type;
      throw ParseException(error.str());
    }
    ros_msg->velocity_type = POSITION_TYPES[vel_type];

    ros_msg->x_vel = ParseDouble(&bin_msg.data_[52]);
    ros_msg->y_vel = ParseDouble(&bin_msg.data_[60]);
    ros_msg->z_vel = ParseDouble(&bin_msg.data_[68]);

    ros_msg->x_vel_sigma = ParseFloat(&bin_msg.data_[76]);
    ros_msg->y_vel_sigma = ParseFloat(&bin_msg.data_[80]);
    ros_msg->z_vel_sigma = ParseFloat(&bin_msg.data_[84]);

    ros_msg->base_station_id.resize(4);
    std::copy(&bin_msg.data_[88], &bin_msg.data_[92], &ros_msg->base_station_id[0]);

    ros_msg->velocity_latency = ParseFloat(&bin_msg.data_[92]);

    ros_msg->diff_age = ParseFloat(&bin_msg.data_[96]);
    ros_msg->solution_age = ParseFloat(&bin_msg.data_[100]);

    ros_msg->num_satellites_tracked = bin_msg.data_[104];
    ros_msg->num_satellites_used_in_solution = bin_msg.data_[105];
    ros_msg->num_gps_and_glonass_l1_used_in_solution = bin_msg.data_[106];
    ros_msg->num_gps_and_glonass_l1_and_l2_used_in_solution = bin_msg.data_[107];
    // Byte 108 is reserved
    GetExtendedSolutionStatusMessage(bin_msg.data_[109],
                                     ros_msg->extended_solution_status);
    // Byte 110 is reserved
    GetSignalsUsed(bin_msg.data_[111], ros_msg->signal_mask);

    return ros_msg;
  }

  BestxyzParser::MessageType BestxyzParser::ParseAscii(const NovatelSentence& sentence) noexcept(false)
  {
    auto msg = std::make_unique<novatel_gps_msgs::msg::NovatelXYZ>();
    HeaderParser h_parser;
    msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of BESTXYZ message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    msg->solution_status = sentence.body[0];
    msg->position_type = sentence.body[1];
    
    valid = valid && ParseDouble(sentence.body[2], msg->x);
    valid = valid && ParseDouble(sentence.body[3], msg->y);
    valid = valid && ParseDouble(sentence.body[4], msg->z);

    valid = valid && ParseFloat(sentence.body[5], msg->x_sigma);
    valid = valid && ParseFloat(sentence.body[6], msg->y_sigma);
    valid = valid && ParseFloat(sentence.body[7], msg->z_sigma);

    msg->velocity_solution_status = sentence.body[8];
    msg->velocity_type = sentence.body[9];

    valid = valid && ParseDouble(sentence.body[10], msg->x_vel);
    valid = valid && ParseDouble(sentence.body[11], msg->y_vel);
    valid = valid && ParseDouble(sentence.body[12], msg->z_vel);

    valid = valid && ParseFloat(sentence.body[13], msg->x_vel_sigma);
    valid = valid && ParseFloat(sentence.body[14], msg->y_vel_sigma);
    valid = valid && ParseFloat(sentence.body[15], msg->z_vel_sigma);

    msg->base_station_id = sentence.body[16];
    valid = valid && ParseFloat(sentence.body[17], msg->velocity_latency);
    
    valid = valid && ParseFloat(sentence.body[18], msg->diff_age);
    valid = valid && ParseFloat(sentence.body[19], msg->solution_age);
    valid = valid && ParseUInt8(sentence.body[20], msg->num_satellites_tracked);
    valid = valid && ParseUInt8(sentence.body[21], msg->num_satellites_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[22], msg->num_gps_and_glonass_l1_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[23], msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

    // skip reserved field
    uint32_t extended_solution_status = 0;
    valid = valid && ParseUInt32(sentence.body[25], extended_solution_status, 16);
    GetExtendedSolutionStatusMessage(
        extended_solution_status, msg->extended_solution_status);

    // skip reserved field
    uint32_t signal_mask = 0;
    valid = valid && ParseUInt32(sentence.body[27], signal_mask, 16);
    GetSignalsUsed(signal_mask, msg->signal_mask);

    if (!valid)
    {
      throw ParseException("Invalid field in BESTXYZ message");
    }

    return msg;
  }
}
