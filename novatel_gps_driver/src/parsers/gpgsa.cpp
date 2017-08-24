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

#include <novatel_gps_driver/parsers/gpgsa.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::GpgsaParser::MESSAGE_NAME = "GPGSA";

uint32_t novatel_gps_driver::GpgsaParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GpgsaParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::GpgsaPtr novatel_gps_driver::GpgsaParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) throw(ParseException)
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

  novatel_gps_msgs::GpgsaPtr msg = boost::make_shared<novatel_gps_msgs::Gpgsa>();
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
