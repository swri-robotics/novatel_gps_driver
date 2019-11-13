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

#include <novatel_gps_driver/parsers/gpgsa.h>

const std::string novatel_gps_driver::GpgsaParser::MESSAGE_NAME = "GPGSA";

uint32_t novatel_gps_driver::GpgsaParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GpgsaParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::GpgsaParser::MessageType novatel_gps_driver::GpgsaParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) noexcept(false)
{
  // Check the length first -- should be 18 elements long
  const size_t LENGTH = 18;
  if (sentence.body.size() != LENGTH)
  {
    std::stringstream error;
    error << "Expected GPGSA length " << LENGTH
          << ", actual length " << sentence.body.size();
    throw ParseException(error.str());
  }

  auto msg = std::make_unique<novatel_gps_msgs::msg::Gpgsa>();
  msg->message_id = sentence.body[0];
  msg->auto_manual_mode = sentence.body[1];
  ParseUInt8(sentence.body[2], msg->fix_mode);
  // Words 3-14 of the sentence are SV IDs. Copy only the non-null strings.
  msg->sv_ids.resize(12, 0);
  size_t n_svs = 0;
  for (std::vector<std::string>::const_iterator id = sentence.body.begin()+3; id < sentence.body.begin()+15; ++id)
  {
    if (! id->empty())
    {
      ParseUInt8(*id, msg->sv_ids[n_svs]);
      ++n_svs;
    }
  }
  msg->sv_ids.resize(n_svs);

  ParseFloat(sentence.body[15], msg->pdop);
  ParseFloat(sentence.body[16], msg->hdop);
  ParseFloat(sentence.body[17], msg->vdop);
  return msg;
}
