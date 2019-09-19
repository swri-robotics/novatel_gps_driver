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

#include <novatel_gps_driver/parsers/gprmc.h>
#include <boost/make_shared.hpp>
#include <swri_string_util/string_util.h>

const std::string novatel_gps_driver::GprmcParser::MESSAGE_NAME = "GPRMC";

uint32_t novatel_gps_driver::GprmcParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GprmcParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::GprmcPtr novatel_gps_driver::GprmcParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) noexcept(false)
{
  // Check the length first; should be 13 elements long for OEM6 & 7,
  // but only 12 elements for OEM4.
  const size_t EXPECTED_LEN_OEM6 = 13;
  const size_t EXPECTED_LEN_OEM4 = 12;

  if (sentence.body.size() != EXPECTED_LEN_OEM4 &&
      sentence.body.size() != EXPECTED_LEN_OEM6)
  {
    std::stringstream error;
    error << "Expected GPRMC lengths = "
          << EXPECTED_LEN_OEM4 << " (for OEM4), "
          << EXPECTED_LEN_OEM6 << " (for OEM6), "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  bool success = true;
  novatel_gps_msgs::GprmcPtr msg = boost::make_shared<novatel_gps_msgs::Gprmc>();
  msg->message_id = sentence.body[0];

  if (sentence.body[1].empty() || sentence.body[1] == "0")
  {
    msg->utc_seconds = 0;
  }
  else
  {
    double utc_float;
    if (swri_string_util::ToDouble(sentence.body[1], utc_float))
    {
      msg->utc_seconds = UtcFloatToSeconds(utc_float);
    }
    else
    {
      throw ParseException("Error parsing UTC seconds in GPRMC log.");
    }
  }

  msg->position_status = sentence.body[2];
  // Check to see whether this message is listed as valid
  success &= (sentence.body[2].compare("A") == 0);
  success &= !(sentence.body[3].empty() || sentence.body[5].empty());

  bool valid = true;

  double latitude = 0.0;
  valid = valid && ParseDouble(sentence.body[3], latitude);
  msg->lat = ConvertDmsToDegrees(latitude);

  double longitude = 0.0;
  valid = valid && ParseDouble(sentence.body[5], longitude);
  msg->lon = ConvertDmsToDegrees(longitude);

  msg->lat_dir = sentence.body[4];
  msg->lon_dir = sentence.body[6];

  valid = valid && ParseFloat(sentence.body[7], msg->speed);
  msg->speed *= KNOTS_TO_MPS;

  valid = valid && ParseFloat(sentence.body[8], msg->track);

  std::string date_str = sentence.body[9];
  if (!date_str.empty())
  {
    msg->date = std::string("20") + date_str.substr(4, 2) +
                std::string("-") + date_str.substr(2, 2) +
                std::string("-") + date_str.substr(0, 2);
  }
  valid = valid && ParseFloat(sentence.body[10], msg->mag_var);
  msg->mag_var_direction = sentence.body[11];
  if (sentence.body.size() == EXPECTED_LEN_OEM6) {
    msg->mode_indicator = sentence.body[12];
  }

  if (!valid)
  {
    was_last_gps_valid_ = false;
    throw ParseException("Error parsing GPRMC message.");
  }

  was_last_gps_valid_ = success;

  return msg;
}

bool novatel_gps_driver::GprmcParser::WasLastGpsValid() const
{
  return was_last_gps_valid_;
}
