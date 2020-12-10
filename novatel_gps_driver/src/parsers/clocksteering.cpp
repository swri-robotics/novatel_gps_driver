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

#include <novatel_gps_driver/parsers/clocksteering.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::ClockSteeringParser::MESSAGE_NAME = "CLOCKSTEERING";

uint32_t novatel_gps_driver::ClockSteeringParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::ClockSteeringParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::ClockSteeringPtr novatel_gps_driver::ClockSteeringParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
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
  novatel_gps_msgs::ClockSteeringPtr msg = boost::make_shared<novatel_gps_msgs::ClockSteering>();

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
