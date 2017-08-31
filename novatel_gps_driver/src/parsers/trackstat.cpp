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

#include <novatel_gps_driver/parsers/trackstat.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::TrackstatParser::MESSAGE_NAME = "TRACKSTAT";

uint32_t novatel_gps_driver::TrackstatParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::TrackstatParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::TrackstatPtr
novatel_gps_driver::TrackstatParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) throw(ParseException)
{
  uint32_t num_chans = ParseUInt32(&bin_msg.data_[12]);
  if (bin_msg.data_.size() != (BINARY_CHANNEL_LENGTH * num_chans) +
                              BINARY_BODY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected trackstat message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }

  uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
  if (solution_status > MAX_SOLUTION_STATUS)
  {
    std::stringstream error;
    error << "Unknown solution status: " << solution_status;
    throw ParseException(error.str());
  }

  novatel_gps_msgs::TrackstatPtr ros_msg = boost::make_shared<novatel_gps_msgs::Trackstat>();
  ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
  uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
  if (pos_type > MAX_DATUM)
  {
    std::stringstream error;
    error << "Unknown position type: " << pos_type;
    throw ParseException(error.str());
  }
  ros_msg->position_type = POSITION_TYPES[pos_type];
  ros_msg->cutoff = ParseFloat(&bin_msg.data_[8]);

  for (int i = 0; i < num_chans; i++)
  {
    size_t chan_offset = BINARY_BODY_LENGTH +
                         i * BINARY_CHANNEL_LENGTH;

    novatel_gps_msgs::TrackstatChannel chan;
    chan.prn = ParseInt16(&bin_msg.data_[chan_offset]);
    chan.glofreq = ParseInt16(&bin_msg.data_[chan_offset+2]);
    chan.ch_tr_status = ParseUInt32(&bin_msg.data_[chan_offset+4]);
    chan.psr = ParseDouble(&bin_msg.data_[chan_offset+8]);
    chan.doppler = ParseFloat(&bin_msg.data_[chan_offset+16]);
    chan.c_no = ParseFloat(&bin_msg.data_[chan_offset+20]);
    chan.locktime = ParseFloat(&bin_msg.data_[chan_offset+24]);
    chan.psr_res = ParseFloat(&bin_msg.data_[chan_offset+28]);
    uint32_t reject = ParseUInt32(&bin_msg.data_[chan_offset+32]);
    switch (reject)
    {
      case 0:
        chan.reject = "GOOD";
        break;
      case 1:
        chan.reject = "BADHEALTH";
        break;
      case 2:
        chan.reject = "OLDEPHEMERIS";
        break;
      case 6:
        chan.reject = "ELEVATIONERROR";
        break;
      case 7:
        chan.reject = "MISCLOSURE";
        break;
      case 8:
        chan.reject = "NODIFFCORR";
        break;
      case 9:
        chan.reject = "NOEPHEMERIS";
        break;
      case 10:
        chan.reject = "INVALIDCODE";
        break;
      case 11:
        chan.reject = "LOCKEDOUT";
        break;
      case 12:
        chan.reject = "LOWPOWER";
        break;
      case 13:
        chan.reject = "OBSL2";
        break;
      case 15:
        chan.reject = "UNKNOWN";
        break;
      case 16:
        chan.reject = "NOIONOCORR";
        break;
      case 17:
        chan.reject = "NOTUSED";
        break;
      case 18:
        chan.reject = "OBSL1";
        break;
      case 19:
        chan.reject = "OBSE1";
        break;
      case 20:
        chan.reject = "OBSL5";
        break;
      case 21:
        chan.reject = "OBSE5";
        break;
      case 22:
        chan.reject = "OBSB2";
        break;
      case 23:
        chan.reject = "OBSB1";
        break;
      case 24:
        chan.reject = "OBSB3";
        break;
      case 25:
        chan.reject = "NOSIGNALMATCH";
        break;
      case 26:
        chan.reject = "SUPPLEMENTARY";
        break;
      case 99:
        chan.reject = "NA";
        break;
      case 100:
        chan.reject = "BAD_INTEGRITY";
        break;
      case 101:
        chan.reject = "LOSSOFLOCK";
        break;
      case 102:
        chan.reject = "NOAMBIGUITY";
        break;
      default:
      {
        std::stringstream error;
        error << "Unexpected channel status: " << reject;
        throw ParseException(error.str());
      }
    }
    chan.psr_weight = ParseFloat(&bin_msg.data_[chan_offset+36]);

    ros_msg->channels.push_back(chan);
  }

  return ros_msg;
}

novatel_gps_msgs::TrackstatPtr
novatel_gps_driver::TrackstatParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.body.size() < ASCII_BODY_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of body fields in TRACKSTAT log: " << sentence.body.size();
    throw ParseException(error.str());
  }

  uint32_t n_channels = 0;
  ParseUInt32(sentence.body[3], n_channels, 10);

  if (sentence.body.size() != ASCII_BODY_FIELDS + n_channels * ASCII_CHANNEL_FIELDS)
  {
    std::stringstream error;
    error << "Size of TRACKSTAT log did not match expected size.";
    throw ParseException(error.str());
  }

  bool valid = true;
  novatel_gps_msgs::TrackstatPtr msg = boost::make_shared<novatel_gps_msgs::Trackstat>();
  msg->solution_status = sentence.body[0];
  msg->position_type = sentence.body[1];
  valid &= ParseFloat(sentence.body[2], msg->cutoff);

  msg->channels.resize(n_channels);
  for (size_t i = 0; i < static_cast<size_t>(n_channels); ++i)
  {
    size_t offset = 4 + i * ASCII_CHANNEL_FIELDS;
    novatel_gps_msgs::TrackstatChannel& channel = msg->channels[i];
    valid &= ParseInt16(sentence.body[offset], channel.prn);
    valid &= ParseInt16(sentence.body[offset+1], channel.glofreq);
    valid &= ParseUInt32(sentence.body[offset+2], channel.ch_tr_status, 16);
    valid &= ParseDouble(sentence.body[offset+3], channel.psr);
    valid &= ParseFloat(sentence.body[offset+4], channel.doppler);
    valid &= ParseFloat(sentence.body[offset+5], channel.c_no);
    valid &= ParseFloat(sentence.body[offset+6], channel.locktime);
    valid &= ParseFloat(sentence.body[offset+7], channel.psr_res);
    channel.reject = sentence.body[offset+8];
    valid &= ParseFloat(sentence.body[offset+9], channel.psr_weight);
  }

  if (!valid)
  {
    std::stringstream error;
    error << "Error parsing TRACKSTAT log.";
    throw ParseException(error.str());
  }
  return msg;
}
