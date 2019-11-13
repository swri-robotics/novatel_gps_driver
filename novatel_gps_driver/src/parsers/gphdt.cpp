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

#include <novatel_gps_driver/parsers/gphdt.h>

#include <boost/lexical_cast.hpp>

const std::string novatel_gps_driver::GphdtParser::MESSAGE_NAME = "GPHDT";

uint32_t novatel_gps_driver::GphdtParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GphdtParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::GphdtParser::MessageType novatel_gps_driver::GphdtParser::ParseAscii(
    const novatel_gps_driver::NmeaSentence& sentence) noexcept(false)
{
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN)
  {
    std::stringstream error;
    error << "Expected GPHDT length = "
          << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  auto msg = std::make_unique<novatel_gps_msgs::msg::Gphdt>();
  msg->message_id = sentence.body[0];

  try
  {
    msg->heading = boost::lexical_cast<double>(sentence.body[1]);
  }
  catch (boost::bad_lexical_cast& e)
  {
    throw ParseException("Error parsing heading as double in GPHDT");
  }

  msg->t = sentence.body[2];
  return msg;
}

