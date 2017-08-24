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

novatel_gps_msgs::GprmcPtr novatel_gps_driver::GprmcParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) throw(ParseException)
{
  // Check the length first -- should be 15 elements long
  const size_t EXPECTED_LEN = 13;
  if (sentence.body.size() != EXPECTED_LEN)
  {
    std::stringstream error;
    error << "Expected GPRMC length = " << EXPECTED_LEN
          << ", actual length = " << sentence.body.size();
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
  msg->mode_indicator = sentence.body[12];

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
