// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <novatel_oem628/novatel_message_parser.h>

#include <limits>
#include <sstream>

#include <ros/ros.h>

#include <swri_string_util/string_util.h>

namespace novatel_oem628
{
  uint32_t CRC32Value(int32_t i)
  {
    int32_t j;
    uint32_t ulCRC;
    ulCRC = static_cast<uint32_t>(i);
    for ( j = 8 ; j > 0; j-- )
    {
      if ( ulCRC & 1 )
        ulCRC = static_cast<uint32_t>(( ulCRC >> 1 ) ^ NOVATEL_CRC32_POLYNOMIAL);
      else
        ulCRC >>= 1;
    }
    return ulCRC;
  }

  uint32_t CalculateBlockCRC32(
      uint32_t ulCount,          // Number of bytes in the data block
      const uint8_t* ucBuffer )  // Data block
  {
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;
    while ( ulCount-- != 0 )
    {
      ulTemp1 = static_cast<uint32_t>(( ulCRC >> 8 ) & 0x00FFFFFFL);
      ulTemp2 = CRC32Value( ((int32_t) ulCRC ^ *ucBuffer++ ) & 0xff );
      ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
  }

  uint8_t NmeaChecksum(const std::string& sentence)
  {
    uint8_t checksum = 0;
    std::string::const_iterator it = sentence.begin();
    for (; it != sentence.end(); ++it)
    {
      checksum ^= *it;
    }
    return checksum;
  }

  size_t get_next_sentence_start(const std::string& str, size_t start_idx)
  {
    size_t nmea_idx = str.find_first_of(NMEA_SENTENCE_FLAG, start_idx);
    size_t novatel_idx = str.find_first_of(NOVATEL_SENTENCE_FLAG, start_idx);
    size_t binary_idx = str.find(NOVATEL_BINARY_SYNC_BYTES, start_idx);

    // Need to check for std::string::npos on the return
    return std::min(std::min(nmea_idx, novatel_idx), binary_idx);
  }

  size_t get_sentence_checksum_start(const std::string& str, size_t start_idx)
  {
    return str.find_first_of('*', start_idx);
  }

  void vectorize_string(
      const std::string& str,
      std::vector<std::string>& vectorized_message,
      char delimiter)
  {
    vectorized_message.clear();
    std::istringstream input(str);
    std::string message_chunk;
    while (std::getline(input, message_chunk, delimiter))
    {
      vectorized_message.push_back(message_chunk);
    }
  }

  bool get_novatel_message_parts(
      const std::string& sentence,
      std::string& message_id,
      std::vector<std::string>& header,
      std::vector<std::string>& body)
  {
    message_id.clear();
    header.clear();
    body.clear();

    std::vector<std::string> vectorized_message;
    vectorize_string(sentence, vectorized_message, ';');

    if (vectorized_message.size() != 2)
    {
      return false;
    }

    vectorize_string(vectorized_message[0], header, ',');

    vectorize_string(vectorized_message[1], body, ',');

    if (!header.empty())
    {
      message_id = header.front();
    }
    else
    {
      return false;
    }
    return true;
  }

  void get_novatel_receiver_status_msg(
      uint32_t status,
      novatel_gps_msgs::NovatelReceiverStatus& receiver_status_msg)
  {
    receiver_status_msg.original_status_code = status;
    receiver_status_msg.error_flag = status & 0x00000001;
    receiver_status_msg.temperature_flag = status & 0x00000002;
    receiver_status_msg.voltage_supply_flag = status & 0x00000004;
    receiver_status_msg.antenna_powered = !(status & 0x00000008);
    receiver_status_msg.antenna_is_open = status & 0x00000020;
    receiver_status_msg.antenna_is_shorted = status & 0x00000040;
    receiver_status_msg.cpu_overload_flag = status & 0x00000080;
    receiver_status_msg.com1_buffer_overrun = status & 0x00000100;
    receiver_status_msg.com2_buffer_overrun = status & 0x00000200;
    receiver_status_msg.com3_buffer_overrun = status & 0x00000400;
    receiver_status_msg.usb_buffer_overrun = status & 0x00000800;
    receiver_status_msg.rf1_agc_flag = status & 0x00008000;
    receiver_status_msg.rf2_agc_flag = status & 0x00020000;
    receiver_status_msg.almanac_flag = status & 0x00040000;
    receiver_status_msg.position_solution_flag = status & 0x00080000;
    receiver_status_msg.position_fixed_flag = status & 0x00100000;
    receiver_status_msg.clock_steering_status_enabled = !(status & 0x00200000);
    receiver_status_msg.clock_model_flag = status & 0x00400000;
    receiver_status_msg.oemv_external_oscillator_flag = status & 0x00800000;
    receiver_status_msg.software_resource_flag = status & 0x01000000;
    receiver_status_msg.aux3_status_event_flag = status & 0x20000000;
    receiver_status_msg.aux2_status_event_flag = status & 0x40000000;
    receiver_status_msg.aux1_status_event_flag = status & 0x80000000;
  }

  void get_extended_solution_status_msg(
      uint32_t status,
      novatel_gps_msgs::NovatelExtendedSolutionStatus& msg)
  {
    msg.original_mask = status;
    msg.advance_rtk_verified = 0x01 & status;
    uint32_t pseudo_iono_correction_mask = (0x0E & status) >> 1;
    switch(pseudo_iono_correction_mask)
    {
      case 0:
        msg.psueorange_iono_correction = "Unknown";
        break;
      case 1:
        msg.psueorange_iono_correction = "Klobuchar Broadcast";
        break;
      case 2:
        msg.psueorange_iono_correction = "SBAS Broadcast";
        break;
      case 3:
        msg.psueorange_iono_correction = "Multi-frequency Computed";
        break;
      case 4:
        msg.psueorange_iono_correction = "PSRDiff Correction";
        break;
      case 5:
        msg.psueorange_iono_correction = "Novatel Blended Iono Value";
        break;
      default:
        msg.psueorange_iono_correction = "Unknown";
        break;
    }
  }

  void get_signals_used(uint32_t mask, novatel_gps_msgs::NovatelSignalMask& msg)
  {
    msg.original_mask = mask;
    msg.gps_L1_used_in_solution = mask & 0x01;
    msg.gps_L2_used_in_solution = mask & 0x02;
    msg.gps_L3_used_in_solution = mask & 0x04;
    msg.glonass_L1_used_in_solution = mask & 0x10;
    msg.glonass_L2_used_in_solution = mask & 0x20;
  }

  bool ParseNovatelBinaryHeader(
      const BinaryMessage& bin_msg,
      novatel_gps_msgs::NovatelMessageHeader& msg)
  {
    switch (bin_msg.header_.message_id_)
    {
      case BESTPOS_BINARY_MESSAGE_ID:
        msg.message_name = "BESTPOS";
        break;
      case BESTVEL_BINARY_MESSAGE_ID:
        msg.message_name = "BESTVEL";
        break;
      case RANGE_BINARY_MESSAGE_ID:
        msg.message_name = "RANGE";
        break;
      case TIME_BINARY_MESSAGE_ID:
        msg.message_name = "TIME";
        break;
      case TRACKSTAT_BINARY_MESSAGE_ID:
        msg.message_name = "TRACKSTAT";
        break;
      default:
        return false;
    }
    // No point in checking whether the port identifier is valid here, because
    // the variable's range is 0-255 and this array has 256 values in it.
    msg.port = PORT_IDENTIFIERS[bin_msg.header_.port_address_];
    msg.sequence_num = bin_msg.header_.sequence_;
    msg.percent_idle_time = bin_msg.header_.idle_time_;
    switch (bin_msg.header_.time_status_)
    {
      case 20:
        msg.gps_time_status = "UNKNOWN";
        break;
      case 60:
        msg.gps_time_status = "APPROXIMATE";
        break;
      case 80:
        msg.gps_time_status = "COARSEADJUSTING";
        break;
      case 100:
        msg.gps_time_status = "COARSE";
        break;
      case 120:
        msg.gps_time_status = "COARSESTEERING";
        break;
      case 130:
        msg.gps_time_status = "FREEWHEELING";
        break;
      case 140:
        msg.gps_time_status = "FINEADJUSTING";
        break;
      case 160:
        msg.gps_time_status = "FINE";
        break;
      case 170:
        msg.gps_time_status = "FINEBACKUPSTEERING";
        break;
      case 180:
        msg.gps_time_status = "FINESTEERING";
        break;
      case 200:
        msg.gps_time_status = "SATTIME";
        break;
      default:
        return false;
    }
    msg.gps_week_num = bin_msg.header_.week_;
    msg.gps_seconds = bin_msg.header_.gpsec_;
    get_novatel_receiver_status_msg(bin_msg.header_.receiver_status_, msg.receiver_status);
    msg.receiver_software_version = bin_msg.header_.receiver_sw_version_;

    return true;
  }

  bool parse_novatel_vectorized_header(
      const std::vector<std::string>& header,
      novatel_gps_msgs::NovatelMessageHeader& msg)
  {
    if (header.size() != NOVATEL_MESSAGE_HEADER_LENGTH)
    {
      ROS_ERROR("Novatel message header size wrong: expect %zu, got %zu",
          NOVATEL_MESSAGE_HEADER_LENGTH,
          header.size());
      return false;
    }

    bool valid = true;

    msg.message_name = header[0];
    msg.port = header[1];
    valid = valid && ParseUInt32(header[2], msg.sequence_num);
    valid = valid && ParseFloat(header[3], msg.percent_idle_time);
    msg.gps_time_status = header[4];
    valid = valid && ParseUInt32(header[5], msg.gps_week_num);
    valid = valid && ParseDouble(header[6], msg.gps_seconds);

    uint32_t receiver_status_code = 0;
    valid = valid && ParseUInt32(header[7], receiver_status_code, 16);
    get_novatel_receiver_status_msg(receiver_status_code, msg.receiver_status);

    valid = valid && ParseUInt32(header[9], msg.receiver_software_version);

    return valid;
  }

  bool ParseNovatelBinaryRangeMessage(const BinaryMessage& bin_msg,
                                      novatel_gps_msgs::RangePtr ros_msg)
  {
    uint32_t num_obs = ParseUInt32(&bin_msg.data_[0]);
    if (bin_msg.data_.size() != (NOVATEL_BINARY_RANGE_OBSERVATION_LENGTH * num_obs) + 4)
    {
      ROS_WARN("Unexpected range message size: %lu", bin_msg.data_.size());
      return false;
    }
    if (!ParseNovatelBinaryHeader(bin_msg, ros_msg->novatel_msg_header))
    {
      return false;
    }

    ros_msg->numb_of_observ = num_obs;
    ros_msg->info.reserve(num_obs);
    for(int i = 0; i < num_obs; i++)
    {
      size_t obs_offset = 4 + i * NOVATEL_BINARY_RANGE_OBSERVATION_LENGTH;

      novatel_gps_msgs::RangeInformation info;

      info.prn_number = ParseUInt16(&bin_msg.data_[obs_offset]);
      info.glofreq = ParseUInt16(&bin_msg.data_[obs_offset+2]);
      info.psr = ParseDouble(&bin_msg.data_[obs_offset+4]);
      info.psr_std = ParseFloat(&bin_msg.data_[obs_offset+12]);
      info.adr = ParseDouble(&bin_msg.data_[obs_offset+16]);
      info.adr_std = ParseFloat(&bin_msg.data_[obs_offset+24]);
      info.dopp = ParseFloat(&bin_msg.data_[obs_offset+28]);
      info.noise_density_ratio = ParseFloat(&bin_msg.data_[obs_offset+32]);
      info.locktime = ParseFloat(&bin_msg.data_[obs_offset+36]);
      info.tracking_status = ParseUInt32(&bin_msg.data_[obs_offset+40]);

      ros_msg->info.push_back(info);
    }
    return true;
  }

  bool ParseNovatelRangeMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::RangePtr msg)
  { 
    if (!parse_novatel_vectorized_header(sentence.header, msg->novatel_msg_header))
    {
      return false;
    }
    if (!ParseInt32(sentence.body[0], msg->numb_of_observ, 10))
    {
      return false;
    }
    int numb_of_observ = msg->numb_of_observ;
    if (sentence.body.size() != 1 + numb_of_observ * NOVATEL_RANGE_BODY_FIELDS)
    {
      return false;
    }
    bool valid = true;
    valid &= ParseInt32(sentence.body[0], msg->numb_of_observ, 10);
    msg->info.resize(numb_of_observ);
    for (int i = 0, index = 0; index < numb_of_observ; i += 10, index++)
    {
      valid &= ParseUInt16(sentence.body[i + 1], msg->info[index].prn_number, 10);
      valid &= ParseUInt16(sentence.body[i + 2], msg->info[index].glofreq, 10);
      valid &= ParseDouble(sentence.body[i + 3], msg->info[index].psr);
      valid &= ParseFloat(sentence.body[i + 4], msg->info[index].psr_std);
      valid &= ParseDouble(sentence.body[i + 5], msg->info[index].adr);
      valid &= ParseFloat(sentence.body[i + 6], msg->info[index].adr_std);
      valid &= ParseFloat(sentence.body[i + 7], msg->info[index].dopp);
      valid &= ParseFloat(sentence.body[i + 8], msg->info[index].noise_density_ratio);
      valid &= ParseFloat(sentence.body[i + 9], msg->info[index].locktime);
      std::string track = "0x" + sentence.body[i + 10]; // This number is in hex
      valid &= ParseUInt32(track, msg->info[index].tracking_status, 16);
    }
    return valid;
  }

  bool ParseNovatelBinaryTrackstatMessage(
      const BinaryMessage& bin_msg,
      novatel_gps_msgs::TrackstatPtr ros_msg)
  {
    uint32_t num_chans = ParseUInt32(&bin_msg.data_[12]);
    if (bin_msg.data_.size() != (NOVATEL_BINARY_TRACKSTAT_CHANNEL_LENGTH * num_chans) +
                                    NOVATEL_BINARY_TRACKSTAT_MIN_LENGTH)
    {
      ROS_WARN("Unexpected trackstat message size: %lu", bin_msg.data_.size());
      return false;
    }

    uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
    if (solution_status > 22)
    {
      ROS_ERROR("Unknown solution status: %u", solution_status);
      return false;
    }
    ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
    uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
    if (pos_type > 74)
    {
      ROS_ERROR("Unknown position type: %u", pos_type);
      return false;
    }
    ros_msg->position_type = POSITION_TYPES[pos_type];
    ros_msg->cutoff = ParseFloat(&bin_msg.data_[8]);

    for (int i = 0; i < num_chans; i++)
    {
      size_t chan_offset = NOVATEL_BINARY_TRACKSTAT_MIN_LENGTH +
          i * NOVATEL_BINARY_TRACKSTAT_CHANNEL_LENGTH;

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
          return false;
      }
      chan.psr_weight = ParseFloat(&bin_msg.data_[chan_offset+36]);

      ros_msg->channels.push_back(chan);
    }

    return true;
  }

  bool ParseNovatelTrackstatMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::TrackstatPtr msg)
  {
    if (!msg)
    {
      return false;
    }

    if (sentence.body.size() < NOVATEL_TRACKSTAT_BODY_FIELDS)
    {
      return false;
    }

    int32_t n_channels = 0;
    ParseInt32(sentence.body[3], n_channels, 10);

    if (sentence.body.size() != NOVATEL_TRACKSTAT_BODY_FIELDS + n_channels * NOVATEL_TRACKSTAT_CHANNEL_FIELDS)
    {
      return false;
    }

    bool valid = true;
    msg->solution_status = sentence.body[0];
    msg->position_type = sentence.body[1];
    valid &= ParseFloat(sentence.body[2], msg->cutoff);

    msg->channels.resize(n_channels);
    for (size_t i = 0; i < static_cast<size_t>(n_channels); ++i)
    {
      size_t offset = 4 + i * NOVATEL_TRACKSTAT_CHANNEL_FIELDS;
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
    return valid;
  }

  bool ParseNovatelCorrectedImuMessage(const NovatelSentence& sentence, 
                                       novatel_gps_msgs::NovatelCorrectedImuDataPtr msg)
  {
    if (!parse_novatel_vectorized_header(sentence.header, msg->novatel_msg_header))
    {
      return false;
    }

    bool valid = true;

    valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
    valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
    valid &= ParseDouble(sentence.body[2], msg->pitch_rate);
    valid &= ParseDouble(sentence.body[3], msg->roll_rate);
    valid &= ParseDouble(sentence.body[4], msg->yaw_rate);
    valid &= ParseDouble(sentence.body[5], msg->lateral_acceleration);
    valid &= ParseDouble(sentence.body[6], msg->longitudinal_acceleration);
    valid &= ParseDouble(sentence.body[7], msg->vertical_acceleration);

    return valid;
  }

  bool ParseNovatelBinaryTimeMessage(
      const BinaryMessage& msg,
      novatel_gps_msgs::TimePtr ros_msg)
  {
    if (msg.data_.size() != NOVATEL_BINARY_TIME_LENGTH)
    {
      ROS_WARN("Unexpected time message size: %lu", msg.data_.size());
      return false;
    }

    uint32_t clock_status = ParseUInt32(&msg.data_[0]);
    switch (clock_status)
    {
      case 0:
        ros_msg->clock_status = "VALID";
        break;
      case 1:
        ros_msg->clock_status = "CONVERGING";
        break;
      case 2:
        ros_msg->clock_status = "ITERATING";
        break;
      case 3:
        ros_msg->clock_status = "INVALID";
        break;
      default:
        ROS_WARN("Unexpected clock status: %u", clock_status);
        return false;
    }
    ros_msg->offset = ParseDouble(&msg.data_[4]);
    ros_msg->offset_std = ParseDouble(&msg.data_[12]);
    ros_msg->utc_offset = ParseDouble(&msg.data_[20]);
    ros_msg->utc_year = ParseUInt32(&msg.data_[28]);
    ros_msg->utc_month = msg.data_[32];
    ros_msg->utc_day = msg.data_[33];
    ros_msg->utc_hour = msg.data_[34];
    ros_msg->utc_minute = msg.data_[35];
    ros_msg->utc_millisecond = msg.data_[36];
    uint32_t utc_status = ParseUInt32(&msg.data_[40]);
    switch (utc_status)
    {
      case 0:
        ros_msg->utc_status = "Invalid";
        break;
      case 1:
        ros_msg->utc_status = "Valid";
        break;
      case 2:
        ros_msg->utc_status = "Warning";
        break;
      default:
        ROS_WARN("Unexpected UTC status: %u", utc_status);
        return false;
    }

    return true;
  }

  bool ParseNovatelTimeMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::TimePtr msg)
  {
    if (sentence.body.size() != NOVATEL_TIME_BODY_FIELDS)
    {
      return false;
    }
    bool valid = true;
    msg->clock_status = sentence.body[0];
    valid &= ParseDouble(sentence.body[1], msg->offset);
    valid &= ParseDouble(sentence.body[2], msg->offset_std);
    valid &= ParseDouble(sentence.body[3], msg->utc_offset);
    valid &= ParseUInt32(sentence.body[4], msg->utc_year, 10);
    valid &= ParseUInt8(sentence.body[5], msg->utc_month, 10);
    valid &= ParseUInt8(sentence.body[6], msg->utc_day, 10);
    valid &= ParseUInt8(sentence.body[7], msg->utc_hour, 10);
    valid &= ParseUInt8(sentence.body[8], msg->utc_minute, 10);
    valid &= ParseUInt32(sentence.body[9], msg->utc_millisecond, 10);
    msg->utc_status = sentence.body[10];

    return valid;
  }

  bool ParseNovatelBinaryVelMessage(const BinaryMessage& bin_msg,
                                    novatel_gps_msgs::NovatelVelocityPtr ros_msg)
  {
    if (bin_msg.data_.size() != NOVATEL_BINARY_BESTVEL_LENGTH)
    {
      ROS_WARN("Unexpected velocity message size: %lu", bin_msg.data_.size());
      return false;
    }
    if (!ParseNovatelBinaryHeader(bin_msg, ros_msg->novatel_msg_header))
    {
      return false;
    }

    uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
    if (solution_status > 22)
    {
      ROS_ERROR("Unknown solution status: %u", solution_status);
      return false;
    }
    ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
    uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
    if (pos_type > 74)
    {
      ROS_ERROR("Unknown position type: %u", pos_type);
      return false;
    }
    ros_msg->velocity_type = POSITION_TYPES[pos_type];
    ros_msg->latency = ParseFloat(&bin_msg.data_[8]);
    ros_msg->age = ParseFloat(&bin_msg.data_[12]);
    ros_msg->horizontal_speed = ParseDouble(&bin_msg.data_[16]);
    ros_msg->track_ground = ParseDouble(&bin_msg.data_[24]);
    ros_msg->vertical_speed = ParseDouble(&bin_msg.data_[32]);

    return true;
  }

  bool ParseNovatelVelMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::NovatelVelocityPtr msg)
  {
    if (!parse_novatel_vectorized_header(sentence.header, msg->novatel_msg_header))
    {
      return false;
    }

    if (sentence.body.size() != NOVATEL_VEL_BODY_FIELDS)
    {
      return false;
    }
    bool valid = true;
    msg->solution_status = sentence.body[0];
    msg->velocity_type = sentence.body[1];
    valid = valid && ParseFloat(sentence.body[2], msg->latency);
    valid = valid && ParseFloat(sentence.body[3], msg->age);
    valid = valid && ParseDouble(sentence.body[4], msg->horizontal_speed);
    valid = valid && ParseDouble(sentence.body[5], msg->track_ground);
    valid = valid && ParseDouble(sentence.body[6], msg->vertical_speed);

    return valid;
  }

  bool parse_binary_novatel_pos_msg(const BinaryMessage& bin_msg,
                                    novatel_gps_msgs::NovatelPositionPtr ros_msg)
  {
    if (bin_msg.data_.size() != NOVATEL_BINARY_BESTPOS_LENGTH)
    {
      ROS_WARN("Unexpected bestpos message length: %lu", bin_msg.data_.size());
      return false;
    }
    if (!ParseNovatelBinaryHeader(bin_msg, ros_msg->novatel_msg_header))
    {
      return false;
    }

    uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
    if (solution_status > 22)
    {
      ROS_ERROR("Unknown solution status: %u", solution_status);
      return false;
    }
    ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
    uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
    if (pos_type > 74)
    {
      ROS_ERROR("Unknown position type: %u", pos_type);
      return false;
    }
    ros_msg->position_type = POSITION_TYPES[pos_type];
    ros_msg->lat = ParseDouble(&bin_msg.data_[8]);
    ros_msg->lon = ParseDouble(&bin_msg.data_[16]);
    ros_msg->height = ParseDouble(&bin_msg.data_[24]);
    ros_msg->undulation = ParseFloat(&bin_msg.data_[32]);
    uint16_t datum_id = ParseUInt16(&bin_msg.data_[36]);
    if (datum_id > 86)
    {
      ROS_ERROR("Unknown datum: %u", datum_id);
      return false;
    }
    ros_msg->datum_id = DATUMS[datum_id];
    ros_msg->lat_sigma = ParseFloat(&bin_msg.data_[40]);
    ros_msg->lon_sigma = ParseFloat(&bin_msg.data_[44]);
    ros_msg->height_sigma = ParseFloat(&bin_msg.data_[48]);
    ros_msg->base_station_id.resize(4);
    std::copy(&bin_msg.data_[52], &bin_msg.data_[56], &ros_msg->base_station_id[0]);
    ros_msg->diff_age = ParseFloat(&bin_msg.data_[56]);
    ros_msg->solution_age = ParseFloat(&bin_msg.data_[60]);
    ros_msg->num_satellites_tracked = bin_msg.data_[64];
    ros_msg->num_satellites_used_in_solution = bin_msg.data_[65];
    ros_msg->num_gps_and_glonass_l1_used_in_solution = bin_msg.data_[66];
    ros_msg->num_gps_and_glonass_l1_and_l2_used_in_solution = bin_msg.data_[67];
    get_extended_solution_status_msg(bin_msg.data_[69],
                                     ros_msg->extended_solution_status);
    get_signals_used(bin_msg.data_[70], ros_msg->signal_mask);

    return true;
  }

  bool parse_novatel_pos_msg(
      const NovatelSentence& sentence,
      novatel_gps_msgs::NovatelPositionPtr msg)
  {
    if (!parse_novatel_vectorized_header(sentence.header, msg->novatel_msg_header))
    {
      return false;
    }

    if (sentence.body.size() != NOVATEL_OMNIHPPOS_BODY_LENGTH)
    {
      return false;
    }

    bool valid = true;

    msg->solution_status = sentence.body[0];
    msg->position_type = sentence.body[1];
    valid = valid && ParseDouble(sentence.body[2], msg->lat);
    valid = valid && ParseDouble(sentence.body[3], msg->lon);
    valid = valid && ParseDouble(sentence.body[4], msg->height);
    valid = valid && ParseFloat(sentence.body[5], msg->undulation);
    msg->datum_id = sentence.body[6];
    valid = valid && ParseFloat(sentence.body[7], msg->lat_sigma);
    valid = valid && ParseFloat(sentence.body[8], msg->lon_sigma);
    valid = valid && ParseFloat(sentence.body[9], msg->height_sigma);
    msg->base_station_id = sentence.body[10];
    valid = valid && ParseFloat(sentence.body[11], msg->diff_age);
    valid = valid && ParseFloat(sentence.body[12], msg->solution_age);
    valid = valid && ParseUInt8(sentence.body[13], msg->num_satellites_tracked);
    valid = valid && ParseUInt8(sentence.body[14], msg->num_satellites_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[15], msg->num_gps_and_glonass_l1_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[16], msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

    // skip reserved field
    uint32_t extended_solution_status = 0;
    valid = valid && ParseUInt32(sentence.body[18], extended_solution_status, 16);
    get_extended_solution_status_msg(
        extended_solution_status, msg->extended_solution_status);

    // skip reserved field
    uint32_t signal_mask = 0;
    valid = valid && ParseUInt32(sentence.body[20], signal_mask, 16);
    get_signals_used(signal_mask, msg->signal_mask);

    return valid;
  }

  int32_t get_binary_message(const std::string& str,
                             size_t start_idx,
                             BinaryMessage& msg)
  {
    if (str.length() < NOVATEL_BINARY_HEADER_LENGTH + 4)
    {
      // The shortest a binary message can be (header + no data + CRC)
      // is 32 bytes, so just return if we don't have at least that many.
      ROS_WARN("Binary message was too short.");
      return -1;
    }

    ROS_DEBUG("Reading binary header.");
    std::copy(&str[start_idx], &str[start_idx+NOVATEL_BINARY_HEADER_LENGTH],
              reinterpret_cast<char*>(&msg.header_));
    uint16_t data_start = static_cast<uint16_t>(NOVATEL_BINARY_HEADER_LENGTH + start_idx);
    uint16_t data_length = msg.header_.message_length_;

    ROS_DEBUG("Data start / length: %u / %u", data_start, data_length);

    if (data_start + data_length + 4 > str.length())
    {
      ROS_DEBUG("Not enough data.");
      return -1;
    }

    ROS_DEBUG("Reading binary message data.");
    msg.data_.resize(data_length);
    std::copy(&str[data_start], &str[data_start+data_length], reinterpret_cast<char*>(&msg.data_[0]));

    ROS_DEBUG("Calculating CRC.");

    uint32_t crc = CalculateBlockCRC32(static_cast<uint32_t>(NOVATEL_BINARY_HEADER_LENGTH) + data_length,
                                       reinterpret_cast<const uint8_t*>(&str[start_idx]));

    ROS_DEBUG("Reading CRC.");
    msg.crc_ = ParseUInt32(reinterpret_cast<const uint8_t*>(&str[data_start+data_length]));

    if (crc != msg.crc_)
    {
      // Invalid CRC
      ROS_DEBUG("Invalid CRC;  Calc: %u    In msg: %u", crc, msg.crc_);
      return 1;
    }

    ROS_DEBUG("Finishing reading binary message.");
    return 0;
  }

  int32_t get_novatel_sentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence)
  {
    sentence.clear();
    size_t checksum_start = get_sentence_checksum_start(str, start_idx);
    if (checksum_start == std::string::npos)
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else if (checksum_start + 8 >= str.size())
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else
    {
      // Compare the checksums
      sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
      std::string checksum_str = str.substr(checksum_start + 1, 8);
      unsigned long checksum = std::strtoul(checksum_str.c_str(), 0, 16);

       if (checksum == ULONG_MAX)
      {
        // Invalid checksum -- strtoul failed
        return 1;
      }
      else if(static_cast<uint32_t>(checksum) == CalculateBlockCRC32(
           static_cast<uint32_t>(sentence.size()),
           reinterpret_cast<const uint8_t*>(sentence.c_str())))
      {
        return 0;
      }
      else
      {
        // Invalid checksum
        return 1;
      }
    }
  }

  int32_t get_nmea_sentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence,
      bool keep_container)
  {
    sentence.clear();
    size_t checksum_start = get_sentence_checksum_start(str, start_idx);
    if (checksum_start == std::string::npos)
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else if (checksum_start + 2 >= str.size())
    {
      // Sentence not complete. Just return.
      return -1;
    }
    else
    {
      // Compare the checksums
      sentence = str.substr(start_idx + 1, checksum_start - start_idx - 1);
      std::string checksum_str = str.substr(checksum_start + 1, 2);
      unsigned long checksum = std::strtoul(checksum_str.c_str(), 0, 16);
      if (checksum == ULONG_MAX)
      {
        // Invalid checksum
        return 1;
      }
      else if(static_cast<uint32_t>(checksum) == NmeaChecksum(sentence))
      {
        if (keep_container)
        {
          sentence.insert(0, "$");
          std::string recreated_checksum_str("*");
          recreated_checksum_str += checksum_str;
          sentence.insert(sentence.end(),
              recreated_checksum_str.begin(),
              recreated_checksum_str.end());
        }
        return 0;
      }
      else
      {
        // Invalid checksum
        return 1;
      }
    }
  }

  bool VectorizeNovatelSentence(
      const std::string& data,
      NovatelSentence& sentence)
  {
    return get_novatel_message_parts(
        data, sentence.id, sentence.header, sentence.body);
  }

  void VectorizeNmeaSentence(
    const std::string& sentence,
    NmeaSentence& vectorized_message)
  {
    vectorize_string(sentence, vectorized_message.body, ',');
    if (!vectorized_message.body.empty())
    {
      vectorized_message.id = vectorized_message.body.front();
    }
  }

  bool extract_complete_sentences(
      const std::string input,
      std::vector<NmeaSentence>& nmea_sentences,
      std::vector<NovatelSentence>& novatel_sentences,
      std::vector<BinaryMessage>& binary_messages,
      std::string& remaining,
      bool keep_nmea_container)
  {
    bool parse_error = false;
    size_t cur_idx = 0;

    size_t sentence_start = 0;
    while(sentence_start != std::string::npos && sentence_start < input.size())
    {
      sentence_start = get_next_sentence_start(input, cur_idx);
      if (sentence_start == std::string::npos)
      {
        remaining.clear();
        break;
      }
      if (input[sentence_start] == NMEA_SENTENCE_FLAG)
      {
        std::string cur_sentence;
        int32_t result = get_nmea_sentence(
            input,
            sentence_start,
            cur_sentence,
            keep_nmea_container);
        if (result == 0)
        {
          nmea_sentences.push_back(NmeaSentence());
          VectorizeNmeaSentence(cur_sentence, nmea_sentences.back());
          cur_idx = sentence_start + 1;
        }
        else if (result < 0)
        {
          // Sentence is not complete, add it to the remaining and break;
          remaining = input.substr(sentence_start);
          break;
        }
        else
        {
          // Sentence had an invalid checksum, just iterate to the next sentence
          cur_idx = sentence_start + 1;
          parse_error = true;
        }
      }
      else if (input[sentence_start] == NOVATEL_SENTENCE_FLAG)
      {
        std::string cur_sentence;
        int32_t result = get_novatel_sentence(input, sentence_start, cur_sentence);
        if (result == 0)
        {
          // Send to parser for testing:
          novatel_sentences.push_back(NovatelSentence());
          if (!VectorizeNovatelSentence(cur_sentence, novatel_sentences.back()))
          {
            novatel_sentences.pop_back();
            parse_error = true;
            ROS_ERROR_THROTTLE(1.0, "Unable to vectorize novatel sentence");
          }
          cur_idx = sentence_start + 1;
        }
        else if (result < 0)
        {
          // Sentence is not complete, add it to the remaining and break;
          remaining = input.substr(sentence_start);
          break;
        }
        else
        {
          // Sentence had an invalid checksum, just iterate to the next sentence
          cur_idx = sentence_start + 1;
          parse_error = true;
        }
      }
      else if (input.substr(sentence_start, NOVATEL_BINARY_SYNC_BYTES.size()) ==
          NOVATEL_BINARY_SYNC_BYTES)
      {
        BinaryMessage cur_msg;
        int32_t result = get_binary_message(input, sentence_start, cur_msg);
        if (result == 0)
        {
          binary_messages.push_back(cur_msg);
          cur_idx = sentence_start + 1;
        }
        else if (result < 0)
        {
          // Sentence is not complete, add it to the remaining and break;
          remaining = input.substr(sentence_start);
          break;
        }
        else
        {
          // Sentence had an invalid checksum, just iterate to the next sentence
          cur_idx = sentence_start + 1;
          ROS_ERROR_THROTTLE(1.0, "Invalid binary message checksum");
          parse_error = true;
        }
      }
      else
      {
        // If for some reason we get here, we'll just iterate to the next message
        cur_idx = sentence_start + 1;
        ROS_ERROR_THROTTLE(1.0, "Unrecognized sentence start: %x", input[sentence_start]);
        parse_error = true;
      }
    }

    return !parse_error;
  }

  double GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences)
  {
    std::vector<NmeaSentence>::const_reverse_iterator iter;
    for (iter = sentences.rbegin(); iter != sentences.rend(); iter++)
    {
      if (iter->id == "GPGGA" || iter->id == "GPRMC")
      {
        if (iter->body.size() > 1)
        {
          if (iter->body[1].empty() || iter->body[1] == "0")
          {
            return 0;
          }
          else
          {
            double utc_float;
            if (swri_string_util::ToDouble(iter->body[1], utc_float))
            {
              return UtcFloatToSeconds(utc_float);
            }
            return 0;
          }
        }
      }
    }

    return 0;
  }

  double UtcFloatToSeconds(double utc_float)
  {
    uint32_t hours = static_cast<uint32_t>(utc_float) / 10000;
    uint32_t minutes = (static_cast<uint32_t>(utc_float) - hours * 10000) / 100;
    double seconds = utc_float -
        static_cast<double>(hours * 10000 + minutes * 100);
    seconds += static_cast<double> (hours * 3600 + minutes * 60);
    return seconds;
  }

  double convert_dms_to_degrees(double dms)
  {
    uint32_t whole_degrees = static_cast<uint32_t>(dms) / 100;
    double minutes = dms - static_cast<double>(whole_degrees * 100);
    // 60 minutes in a degree
    double degrees = static_cast<double>(whole_degrees) + minutes / 60.0;
    return degrees;
  }

  NmeaMessageParseResult parse_vectorized_gpgga_message(
      const std::vector<std::string>& vec,
      novatel_gps_msgs::GpggaPtr msg)
  {
    // Check the length first -- should be 15 elements long
    const size_t MAX_LEN = 15;
    const size_t MIN_LEN = 14;
    if (vec.size() > MAX_LEN || vec.size() < MIN_LEN)
    {
      ROS_ERROR("Expected GPGGA length %zu <= length <= %zu, actual length = %zu",
          MIN_LEN,
          MAX_LEN,
          vec.size());
      return ParseFailed;
    }

    msg->message_id = vec[0];

    if (vec[1].empty() || vec[1] == "0")
    {
      msg->utc_seconds = 0;
    }
    else
    {
      double utc_float;
      if (swri_string_util::ToDouble(vec[1], utc_float))
      {
        msg->utc_seconds = UtcFloatToSeconds(utc_float);
      }
      else
      {
        return ParseFailed;
      }
    }

    bool valid = true;

    double latitude = 0.0;
    valid = valid && ParseDouble(vec[2], latitude);
    msg->lat = convert_dms_to_degrees(latitude);

    double longitude = 0.0;
    valid = valid && ParseDouble(vec[4], longitude);
    msg->lon = convert_dms_to_degrees(longitude);

    msg->lat_dir = vec[3];
    msg->lon_dir = vec[5];
    valid = valid && ParseUInt32(vec[6], msg->gps_qual);
    valid = valid && ParseUInt32(vec[7], msg->num_sats);

    valid = valid && ParseFloat(vec[8], msg->hdop);
    valid = valid && ParseFloat(vec[9], msg->alt);
    msg->altitude_units = vec[10];
    valid = valid && ParseFloat(vec[11], msg->undulation);
    msg->undulation_units = vec[12];
    valid = valid && ParseUInt32(vec[13], msg->diff_age);
    if (vec.size() == MAX_LEN)
    {
      msg->station_id = vec[14];
    }
    else
    {
      msg->station_id = "";
    }

    if (!valid)
    {
      return ParseFailed;
    }

    // Check for actual lat and lon data
    if (vec[2].empty() || vec[4].empty())
    {
      // No Lat or Lon data, return false;
      return ParseSucceededAndGpsDataNotValid;
    }

    return ParseSucceededAndGpsDataValid;
  }

  NmeaMessageParseResult parse_vectorized_gpgsa_message(
      const std::vector<std::string>& vec,
      novatel_gps_msgs::GpgsaPtr msg)
  {
    // Check the length first -- should be 18 elements long
    const size_t LENGTH = 18;
    if (vec.size() != LENGTH)
    {
      ROS_ERROR("Expected GPGSA length %zu, actual length = %zu",
          LENGTH,
          vec.size());
      return ParseFailed;
    }

    msg->message_id = vec[0];
    msg->auto_manual_mode = vec[1];
    ParseUInt8(vec[2], msg->fix_mode);
    // Words 3-14 of the sentence are SV IDs. Copy only the non-null strings.
    msg->sv_ids.resize(12, 0);
    size_t n_svs = 0;
    for (std::vector<std::string>::const_iterator id = vec.begin()+3; id < vec.begin()+15; ++id)
    {
      if (! id->empty())
      {
        ParseUInt8(*id, msg->sv_ids[n_svs]);
        ++n_svs;
      }
    }
    msg->sv_ids.resize(n_svs);

    ParseFloat(vec[15], msg->pdop);
    ParseFloat(vec[16], msg->hdop);
    ParseFloat(vec[17], msg->vdop);
    return ParseSucceededAndGpsDataValid;
  }

  NmeaMessageParseResult ParseVectorizedGpgsvMessage(
      const std::vector<std::string>& vec,
      novatel_gps_msgs::GpgsvPtr msg)
  {
    const size_t MIN_LENGTH = 4;
    // Check that the message is at least as long as a a GPGSV with no satellites
    if (vec.size() < MIN_LENGTH)
    {
      ROS_ERROR("Expected GPGSV length >=%zu, actual length = %zu",
          MIN_LENGTH,
          vec.size());
      return ParseFailed;
    }
    msg->message_id = vec[0];
    bool valid = true;
    valid &= ParseUInt8(vec[1], msg->n_msgs);
    valid &= (msg->n_msgs <= 9);  // Check that number of messages <= 9
    valid &= ParseUInt8(vec[2], msg->msg_number);
    valid &= (msg->msg_number <= msg->n_msgs);  // Check that this message is within the sequence range
    valid &= ParseUInt8(vec[3], msg->n_satellites);

    // Figure out how many satellites should be described in this sentence
    size_t n_sats_in_sentence = 4;
    if (msg->msg_number == msg->n_msgs)
    {
      n_sats_in_sentence = msg->n_satellites % 4;
    }
    if (n_sats_in_sentence == 0)
    {
      n_sats_in_sentence = 4;
    }
    // Check that the sentence is the right length for the number of satellites
    size_t expected_length = MIN_LENGTH + 4 * n_sats_in_sentence;
    if (!valid || (vec.size() != expected_length && vec.size() != expected_length -1))
    {
      std::stringstream ss;
      for (size_t i = 0; i < vec.size(); ++i)
      {
        ss << vec[i];
        if ((i+1) < vec.size())
        {
          ss << ",";
        }
      }
      ROS_ERROR("Expected GPGSV length = %zu for message with %zu satellites, actual length = %zu\n%s",
          expected_length,
          n_sats_in_sentence,
          vec.size(),
          ss.str().c_str());
      return ParseFailed;
    }
    msg->satellites.resize(n_sats_in_sentence);
    for (size_t sat = 0, index=MIN_LENGTH; sat < n_sats_in_sentence; ++sat, index += 4)
    {
      valid &= ParseUInt8(vec[index], msg->satellites[sat].prn);
      valid &= ParseUInt8(vec[index + 1], msg->satellites[sat].elevation);
      valid &= ParseUInt16(vec[index + 2], msg->satellites[sat].azimuth);
      if ((index + 3) >= vec.size() || vec[index + 3].empty())
      {
        msg->satellites[sat].snr = -1;
      }
      else
      {
        uint8_t snr;
        valid &= ParseUInt8(vec[index + 3], snr);
        msg->satellites[sat].snr = static_cast<int8_t>(snr);
      }
    }
    return (valid ? ParseSucceededAndGpsDataValid : ParseFailed);
  }

  NmeaMessageParseResult parse_vectorized_gprmc_message(
      const std::vector<std::string>& vec,
      novatel_gps_msgs::GprmcPtr msg)
  {
    // Check the length first -- should be 15 elements long
    const size_t EXPECTED_LEN = 13;
    if (vec.size() != EXPECTED_LEN)
    {
      ROS_ERROR("Expected GPRMC length = %zu, actual length = %zu",
          EXPECTED_LEN,
          vec.size());
      return ParseFailed;
    }

    bool success = true;
    msg->message_id = vec[0];

    if (vec[1].empty() || vec[1] == "0")
    {
      msg->utc_seconds = 0;
    }
    else
    {
      double utc_float;
      if (swri_string_util::ToDouble(vec[1], utc_float))
      {
        msg->utc_seconds = UtcFloatToSeconds(utc_float);
      }
      else
      {
        return ParseFailed;
      }
    }

    msg->position_status = vec[2];
    // Check to see whether this message is listed as valid
    success &= (vec[2].compare("A") == 0);
    success &= !(vec[3].empty() || vec[5].empty());

    bool valid = true;

    double latitude = 0.0;
    valid = valid && ParseDouble(vec[3], latitude);
    msg->lat = convert_dms_to_degrees(latitude);

    double longitude = 0.0;
    valid = valid && ParseDouble(vec[5], longitude);
    msg->lon = convert_dms_to_degrees(longitude);

    msg->lat_dir = vec[4];
    msg->lon_dir = vec[6];

    valid = valid && ParseFloat(vec[7], msg->speed);
    msg->speed *= KNOTS_TO_MPS;

    valid = valid && ParseFloat(vec[8], msg->track);

    std::string date_str = vec[9];
    if (!date_str.empty())
    {
      msg->date = std::string("20") + date_str.substr(4, 2) +
          std::string("-") + date_str.substr(2, 2) +
          std::string("-") + date_str.substr(0, 2);
    }
    valid = valid && ParseFloat(vec[10], msg->mag_var);
    msg->mag_var_direction = vec[11];
    msg->mode_indicator = vec[12];

    if (!success)
    {
      // The message did not provide a valid position measurement
      return ParseSucceededAndGpsDataNotValid;
    }
    return ParseSucceededAndGpsDataValid;
  }

  void get_gps_fix_message(
      const novatel_gps_msgs::Gprmc& gprmc,
      const novatel_gps_msgs::Gpgga& gpgga,
      gps_common::GPSFixPtr gps_fix)
  {
    gps_fix->header.stamp = gpgga.header.stamp;
    gps_fix->altitude = gpgga.alt;
    // gps_fix.climb // We don't get this information from the receiver
    // gps_fix.dip // We don't get this information from the receiver
    // gps_fix.err // To be filled in later from BestPos messages
    // gps_fix.err_climb // We don't get this information from the receiver
    // gps_fix.err_dip // We don't get this information from the receiver
    // gps_fix.err_horz // To be filled in later from BestPos messages
    // gps_fix.err_pitch // We don't get this information from the receiver
    // gps_fix.err_roll // We don't get this information from the receiver
    // gps_fix.err_speed = ERR_INIT_HIGH;
    // gps_fix.err_time = ERR_INIT_HIGH;
    // gps_fix.err_track = ERR_INIT_HIGH;
    // gps_fix.err_vert = ERR_INIT_HIGH;
    // gps_fix.gdop = ERR_INIT_HIGH
    gps_fix->hdop = gpgga.hdop;
    gps_fix->latitude = gprmc.lat;
    if (gpgga.lat_dir == "S")
    {
      gps_fix->latitude *= -1;
    }

    gps_fix->longitude = gprmc.lon;
    if (gpgga.lon_dir == "W")
    {
      gps_fix->longitude *= -1;
    }
    // gps_fix.pdop = ERR_INIT_HIGH;
    // gps_fix.pitch = 0.0;
    // gps_fix.roll = 0.0;
    gps_fix->speed = gprmc.speed;
    // gps_fix.tdop = ERR_INIT_HIGH;
    gps_fix->time = gpgga.utc_seconds;
    gps_fix->track = gprmc.track;
    // gps_fix.vdop = ERR_INIT_HIGH;

    gps_fix->status.status = gps_common::GPSStatus::STATUS_FIX;
    gps_fix->status.satellites_used = static_cast<uint16_t>(gpgga.num_sats);

  }

  double ParseDouble(const uint8_t* buffer)
  {
    double x;
    std::copy(buffer, buffer + sizeof(double), reinterpret_cast<uint8_t*>(&x));
    return x;
  }

  bool ParseDouble(const std::string& string, double& value)
  {
    return swri_string_util::ToDouble(string, value) || string.empty();
  }

  float ParseFloat(const uint8_t* buffer)
  {
    float x;
    std::copy(buffer, buffer + sizeof(float), reinterpret_cast<uint8_t*>(&x));
    return x;
  }

  bool ParseFloat(const std::string& string, float& value)
  {
    return swri_string_util::ToFloat(string, value) || string.empty();
  }

  int16_t ParseInt16(const uint8_t* buffer)
  {
    int16_t number;
    std::copy(buffer, buffer+2, reinterpret_cast<uint8_t*>(&number));
    return number;
  }

  bool ParseInt16(const std::string& string, int16_t& value, int32_t base)
  {
    value = 0;
    if (string.empty())
    {
      return true;
    }

    int32_t tmp;
    if (swri_string_util::ToInt32(string, tmp, base) &&
        tmp <= std::numeric_limits<int16_t>::max() &&
        tmp >= std::numeric_limits<int16_t>::min())
    {
      value = static_cast<int16_t>(tmp);
      return true;
    }

    return false;
  }

  int32_t ParseInt32(const uint8_t* buffer)
  {
    int32_t number;
    std::copy(buffer, buffer+4, reinterpret_cast<uint8_t*>(&number));
    return number;
  }

  bool ParseInt32(const std::string& string, int32_t& value, int32_t base)
  {
    return swri_string_util::ToInt32(string, value, base) || string.empty();
  }

  uint32_t ParseUInt32(const uint8_t* buffer)
  {
    uint32_t number;
    std::copy(buffer, buffer+4, reinterpret_cast<uint8_t*>(&number));
    return number;
  }

  bool ParseUInt32(const std::string& string, uint32_t& value, int32_t base)
  {
    return swri_string_util::ToUInt32(string, value, base) || string.empty();
  }

  bool ParseUInt8(const std::string& string, uint8_t& value, int32_t base)
  {
    value = 0;
    if (string.empty())
    {
      return true;
    }

    uint32_t tmp;
    if (swri_string_util::ToUInt32(string, tmp, base) && tmp <= std::numeric_limits<uint8_t>::max())
    {
      value = static_cast<uint8_t>(tmp);
      return true;
    }

    return false;
  }

  uint16_t ParseUInt16(const uint8_t* buffer)
  {
    uint16_t number;
    std::copy(buffer, buffer+2, reinterpret_cast<uint8_t*>(&number));
    return number;
  }

  bool ParseUInt16(const std::string& string, uint16_t& value, int32_t base)
  {
    value = 0;
    if (string.empty())
    {
      return true;
    }

    uint32_t tmp;
    if (swri_string_util::ToUInt32(string, tmp, base) && tmp <= std::numeric_limits<uint16_t>::max())
    {
      value = static_cast<uint16_t>(tmp);
      return true;
    }

    return false;
  }
}
