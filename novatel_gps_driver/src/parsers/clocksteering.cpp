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
#include <vector>

#include <novatel_gps_driver/parsers/clocksteering.h>

const std::string novatel_gps_driver::ClockSteeringParser::MESSAGE_NAME = "CLOCKSTEERING";

namespace
{
  // Binary enum values, in order, as the ASCII log names them.
  const std::vector<std::string> CLOCK_SOURCES = {"INTERNAL", "EXTERNAL"};
  const std::vector<std::string> STEERING_STATES = {
    "FIRST_ORDER", "SECOND_ORDER", "CALIBRATE_HIGH", "CALIBRATE_LOW", "CALIBRATE_CENTER"};
}

uint32_t novatel_gps_driver::ClockSteeringParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::ClockSteeringParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::ClockSteeringParser::MessageType novatel_gps_driver::ClockSteeringParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected CLOCKSTEERING message length: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  auto msg = std::make_unique<novatel_gps_msgs::msg::ClockSteering>();

  uint32_t source = ParseUInt32(&bin_msg.data_[0]);
  if (source >= CLOCK_SOURCES.size())
  {
    std::stringstream error;
    error << "Unexpected clock source in CLOCKSTEERING: " << source;
    throw ParseException(error.str());
  }
  msg->source = CLOCK_SOURCES[source];

  uint32_t steering_state = ParseUInt32(&bin_msg.data_[4]);
  if (steering_state >= STEERING_STATES.size())
  {
    std::stringstream error;
    error << "Unexpected steering state in CLOCKSTEERING: " << steering_state;
    throw ParseException(error.str());
  }
  msg->steering_state = STEERING_STATES[steering_state];

  msg->period = ParseUInt32(&bin_msg.data_[8]);
  msg->pulse_width = ParseDouble(&bin_msg.data_[12]);
  msg->bandwidth = ParseDouble(&bin_msg.data_[20]);
  msg->slope = ParseFloat(&bin_msg.data_[28]);
  msg->offset = ParseDouble(&bin_msg.data_[32]);
  msg->drift_rate = ParseDouble(&bin_msg.data_[40]);

  return msg;
}

novatel_gps_driver::ClockSteeringParser::MessageType novatel_gps_driver::ClockSteeringParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  const size_t MIN_LENGTH = 8;
  // Check that the message is at least as long as a a ClockSteering with no satellites
  if (sentence.body.size() != MIN_LENGTH)
  {
    std::stringstream error;
    error << "Expected ClockSteering length >= " << MIN_LENGTH
          << ", actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }
  auto msg = std::make_unique<novatel_gps_msgs::msg::ClockSteering>();

  msg->source = sentence.body[0];
  msg->steering_state = sentence.body[1];

  if (!ParseUInt32(sentence.body[2], msg->period))
  {
    throw ParseException("Error parsing period in ClockSteering.");
  }

  if (!ParseDouble(sentence.body[3], msg->pulse_width))
  {
    throw ParseException("Error parsing pulse_width in ClockSteering.");
  }

  if (!ParseDouble(sentence.body[4], msg->bandwidth))
  {
    throw ParseException("Error parsing bandwidth in ClockSteering.");
  }

  if (!ParseFloat(sentence.body[5], msg->slope))
  {
    throw ParseException("Error parsing slope in ClockSteering.");
  }

  if (!ParseDouble(sentence.body[6], msg->offset))
  {
    throw ParseException("Error parsing offset in ClockSteering.");
  }

  if (!ParseDouble(sentence.body[7], msg->drift_rate))
  {
    throw ParseException("Error parsing drift_rate in ClockSteering.");
  }

  return msg;
}
