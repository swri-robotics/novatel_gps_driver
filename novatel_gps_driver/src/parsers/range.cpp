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

#include <novatel_gps_driver/parsers/range.h>
#include <novatel_gps_driver/parsers/header.h>

const std::string novatel_gps_driver::RangeParser::MESSAGE_NAME = "RANGE";

uint32_t novatel_gps_driver::RangeParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::RangeParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::RangeParser::MessageType
novatel_gps_driver::RangeParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  uint32_t num_obs = ParseUInt32(&bin_msg.data_[0]);
  if (bin_msg.data_.size() != (BINARY_OBSERVATION_SIZE * num_obs) + 4)
  {
    std::stringstream error;
    error << "Unexpected range message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  auto ros_msg = std::make_unique<novatel_gps_msgs::msg::Range>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = "RANGE";

  ros_msg->numb_of_observ = num_obs;
  ros_msg->info.reserve(num_obs);
  for(int i = 0; i < num_obs; i++)
  {
    size_t obs_offset = 4 + i * BINARY_OBSERVATION_SIZE;

    novatel_gps_msgs::msg::RangeInformation info;

    info.prn_number = ParseUInt16(&bin_msg.data_[obs_offset]);
    info.glofreq = ParseUInt16(&bin_msg.data_[obs_offset+2]);
    info.psr = ParseDouble(&bin_msg.data_[obs_offset+4]);
    info.psr_std = ParseFloat(&bin_msg.data_[obs_offset+12]);
    info.adr = ParseDouble(&bin_msg.data_[obs_offset+16]);
    info.adr_std = ParseFloat(&bin_msg.data_[obs_offset+24]);
    info.dopp = ParseFloat(&bin_msg.data_[obs_offset+28]);
    info.noise_density_ratio = ParseFloat(&bin_msg.data_[obs_offset+32]);
    info.locktime = ParseFloat(&bin_msg.data_[obs_offset+36]);
    info.tracking_status = ParseUInt32(&bin_msg.data_[obs_offset+40]);

    ros_msg->info.push_back(info);
  }
  return ros_msg;
}

novatel_gps_driver::RangeParser::MessageType
novatel_gps_driver::RangeParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  auto msg = std::make_unique<novatel_gps_msgs::msg::Range>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);
  if (!ParseInt32(sentence.body[0], msg->numb_of_observ, 10))
  {
    std::stringstream error;
    error << "Unable to parse number of observations in RANGE log.";
    throw ParseException(error.str());
  }
  uint32_t numb_of_observ = static_cast<uint32_t>(msg->numb_of_observ);
  if (sentence.body.size() != 1 + numb_of_observ * ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Did not find expected number of observations in RANGE log.";
    throw ParseException(error.str());
  }
  bool valid = true;
  valid &= ParseInt32(sentence.body[0], msg->numb_of_observ, 10);
  msg->info.resize(numb_of_observ);
  for (int i = 0, index = 0; index < numb_of_observ; i += 10, index++)
  {
    valid &= ParseUInt16(sentence.body[i + 1], msg->info[index].prn_number, 10);
    valid &= ParseUInt16(sentence.body[i + 2], msg->info[index].glofreq, 10);
    valid &= ParseDouble(sentence.body[i + 3], msg->info[index].psr);
    valid &= ParseFloat(sentence.body[i + 4], msg->info[index].psr_std);
    valid &= ParseDouble(sentence.body[i + 5], msg->info[index].adr);
    valid &= ParseFloat(sentence.body[i + 6], msg->info[index].adr_std);
    valid &= ParseFloat(sentence.body[i + 7], msg->info[index].dopp);
    valid &= ParseFloat(sentence.body[i + 8], msg->info[index].noise_density_ratio);
    valid &= ParseFloat(sentence.body[i + 9], msg->info[index].locktime);
    std::string track = "0x" + sentence.body[i + 10]; // This number is in hex
    valid &= ParseUInt32(track, msg->info[index].tracking_status, 16);
  }
  if (!valid)
  {
    throw ParseException("Error parsing RANGE log.");
  }
  return msg;
}
