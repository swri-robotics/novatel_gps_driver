// *****************************************************************************
//
// Copyright (C) 2016 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <novatel_gps_driver/parsers/gpgga.h>
#include <boost/make_shared.hpp>

#include <swri_string_util/string_util.h>

const std::string novatel_gps_driver::GpggaParser::MESSAGE_NAME = "GPGGA";

uint32_t novatel_gps_driver::GpggaParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GpggaParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::GpggaPtr novatel_gps_driver::GpggaParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) throw(ParseException)
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

  novatel_gps_msgs::GpggaPtr msg = boost::make_shared<novatel_gps_msgs::Gpgga>();

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
    throw ParseException("GPGGA log was invalid.");
  }

  // Check for actual lat and lon data
  if (sentence.body[2].empty() || sentence.body[4].empty())
  {
    // No Lat or Lon data, return false;
    was_last_gps_valid_ = false;
  }

  was_last_gps_valid_ = true;

  return msg;
}

bool novatel_gps_driver::GpggaParser::WasLastGpsValid() const
{
  return was_last_gps_valid_;
}
