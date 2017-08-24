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

#include <novatel_gps_driver/parsers/gpgsv.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::GpgsvParser::MESSAGE_NAME = "GPGSV";

uint32_t novatel_gps_driver::GpgsvParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GpgsvParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::GpgsvPtr novatel_gps_driver::GpgsvParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) throw(ParseException)
{
  const size_t MIN_LENGTH = 4;
  // Check that the message is at least as long as a a GPGSV with no satellites
  if (sentence.body.size() < MIN_LENGTH)
  {
    std::stringstream error;
    error << "Expected GPGSV length >= " << MIN_LENGTH
          << ", actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::GpgsvPtr msg = boost::make_shared<novatel_gps_msgs::Gpgsv>();
  msg->message_id = sentence.body[0];
  if (!ParseUInt8(sentence.body[1], msg->n_msgs))
  {
    throw new ParseException("Error parsing n_msgs in GPGSV.");
  }
  if (msg->n_msgs > 9)  // Check that number of messages <= 9
  {
    std::stringstream error;
    error << "n_msgs in GPGSV was too large (" << msg->n_msgs << ").";
    throw ParseException(error.str());
  }

  if (!ParseUInt8(sentence.body[2], msg->msg_number))
  {
    throw ParseException("Error parsing msg_number in GPGSV.");
  }
  if (msg->msg_number > msg->n_msgs)  // Check that this message is within the sequence range
  {
    std::stringstream error;
    error << "msg_number in GPGSV was larger than n_msgs (" << msg->msg_number << " > " << msg->n_msgs << ").";
    throw ParseException(error.str());
  }
  if (!ParseUInt8(sentence.body[3], msg->n_satellites))
  {
    throw ParseException("Error parsing n_satellites in GPGSV.");
  }


  // Figure out how many satellites should be described in this sentence
  size_t n_sats_in_sentence = 4;
  if (msg->msg_number == msg->n_msgs)
  {
    n_sats_in_sentence = msg->n_satellites % static_cast<uint8_t>(4);
  }
  if (n_sats_in_sentence == 0)
  {
    n_sats_in_sentence = 4;
  }
  // Check that the sentence is the right length for the number of satellites
  size_t expected_length = MIN_LENGTH + 4 * n_sats_in_sentence;
  if (sentence.body.size() != expected_length && sentence.body.size() != expected_length -1)
  {
    std::stringstream ss;
    for (size_t i = 0; i < sentence.body.size(); ++i)
    {
      ss << sentence.body[i];
      if ((i+1) < sentence.body.size())
      {
        ss << ",";
      }
    }
    std::stringstream error;
    error << "Expected GPGSV length = " << expected_length << " for message with "
          << n_sats_in_sentence << " satellites, actual length = "
          << sentence.body.size() << "\n" << ss.str().c_str();
    throw ParseException(error.str());
  }
  msg->satellites.resize(n_sats_in_sentence);
  for (size_t sat = 0, index=MIN_LENGTH; sat < n_sats_in_sentence; ++sat, index += 4)
  {
    if (!ParseUInt8(sentence.body[index], msg->satellites[sat].prn))
    {
      std::stringstream error;
      error << "Error parsing prn for satellite " << sat << " in GPGSV.";
      throw ParseException(error.str());
    }

    float elevation;
    if (!ParseFloat(sentence.body[index + 1], elevation))
    {
      std::stringstream error;
      error << "Error parsing elevation for satellite " << sat << " in GPGSV.";
      throw ParseException(error.str());
    }
    msg->satellites[sat].elevation = static_cast<uint8_t>(elevation);

    float azimuth;
    if (!ParseFloat(sentence.body[index + 2], azimuth))
    {
      std::stringstream error;
      error << "Error parsing azimuth for satellite " << sat << " in GPGSV.";
      throw ParseException(error.str());
    }
    msg->satellites[sat].azimuth = static_cast<uint16_t>(azimuth);

    if ((index + 3) >= sentence.body.size() || sentence.body[index + 3].empty())
    {
      msg->satellites[sat].snr = -1;
    }
    else
    {
      uint8_t snr;
      if (!ParseUInt8(sentence.body[index + 3], snr))
      {
        std::stringstream error;
        error << "Error parsing snr for satellite " << sat << " in GPGSV.";
        throw ParseException(error.str());
      }

      msg->satellites[sat].snr = static_cast<int8_t>(snr);
    }
  }
  return msg;
}
