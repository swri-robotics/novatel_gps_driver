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

#include <novatel_gps_driver/parsers/gpgsv.h>

const std::string novatel_gps_driver::GpgsvParser::MESSAGE_NAME = "GPGSV";

uint32_t novatel_gps_driver::GpgsvParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GpgsvParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::GpgsvParser::MessageType novatel_gps_driver::GpgsvParser::ParseAscii(
    const novatel_gps_driver::NmeaSentence& sentence) noexcept(false)
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
  auto msg = std::make_unique<novatel_gps_msgs::msg::Gpgsv>();
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
  // Check that the sentence is the right length for the number of satellites
  size_t expected_length = MIN_LENGTH + 4 * n_sats_in_sentence;
  if (n_sats_in_sentence == 0)
  {
    // Even if the number of sats is 0, the message will still have enough
    // blank fields for 1 satellite.
    expected_length += 4;
  }
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
