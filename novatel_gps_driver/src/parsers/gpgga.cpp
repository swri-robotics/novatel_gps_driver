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

#include <novatel_gps_driver/parsers/gpgga.h>

#include <boost/lexical_cast.hpp>

const std::string novatel_gps_driver::GpggaParser::MESSAGE_NAME = "GPGGA";

uint32_t novatel_gps_driver::GpggaParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GpggaParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::GpggaParser::MessageType novatel_gps_driver::GpggaParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) noexcept(false)
{
  // Check the length first -- should be 15 elements long
  const size_t MAX_LEN = 15;
  const size_t MIN_LEN = 14;
  if (sentence.body.size() > MAX_LEN || sentence.body.size() < MIN_LEN)
  {
    std::stringstream error;
    error << "Expected GPGGA length " << MIN_LEN << "  <= length <= "
          << MAX_LEN << ", actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  auto msg = std::make_unique<novatel_gps_msgs::msg::Gpgga>();

  msg->message_id = sentence.body[0];

  if (sentence.body[1].empty() || sentence.body[1] == "0")
  {
    msg->utc_seconds = 0;
  }
  else
  {
    try
    {
      msg->utc_seconds = boost::lexical_cast<double>(sentence.body[1]);
    }
    catch (boost::bad_lexical_cast& e)
    {
      throw ParseException("Error parsing UTC seconds in GPGGA");
    }
  }

  bool valid = true;

  double latitude = 0.0;
  valid = valid && ParseDouble(sentence.body[2], latitude);
  msg->lat = ConvertDmsToDegrees(latitude);

  double longitude = 0.0;
  valid = valid && ParseDouble(sentence.body[4], longitude);
  msg->lon = ConvertDmsToDegrees(longitude);

  msg->lat_dir = sentence.body[3];
  msg->lon_dir = sentence.body[5];
  valid = valid && ParseUInt32(sentence.body[6], msg->gps_qual);
  valid = valid && ParseUInt32(sentence.body[7], msg->num_sats);

  valid = valid && ParseFloat(sentence.body[8], msg->hdop);
  valid = valid && ParseFloat(sentence.body[9], msg->alt);
  msg->altitude_units = sentence.body[10];
  valid = valid && ParseFloat(sentence.body[11], msg->undulation);
  msg->undulation_units = sentence.body[12];
  valid = valid && ParseUInt32(sentence.body[13], msg->diff_age);
  if (sentence.body.size() == MAX_LEN)
  {
    msg->station_id = sentence.body[14];
  }
  else
  {
    msg->station_id = "";
  }

  if (!valid)
  {
    was_last_gps_valid_ = false;
    throw ParseException("GPGGA log was invalid.");
  }

  // If we got this far, we successfully parsed the message and will consider
  // it valid
  was_last_gps_valid_ = true;

  return msg;
}

bool novatel_gps_driver::GpggaParser::WasLastGpsValid() const
{
  return was_last_gps_valid_;
}
