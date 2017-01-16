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

    // Need to check for std::string::npos on the return
    return std::min(nmea_idx, novatel_idx);
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
      novatel_msgs::NovatelReceiverStatus& receiver_status_msg)
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
      novatel_msgs::NovatelExtendedSolutionStatus& msg)
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

  void get_signals_used(uint32_t mask, novatel_msgs::NovatelSignalMask& msg)
  {
    msg.original_mask = mask;
    msg.gps_L1_used_in_solution = mask & 0x01;
    msg.gps_L2_used_in_solution = mask & 0x02;
    msg.gps_L3_used_in_solution = mask & 0x04;
    msg.glonass_L1_used_in_solution = mask & 0x10;
    msg.glonass_L2_used_in_solution = mask & 0x20;
  }

  bool parse_novatel_vectorized_header(
      const std::vector<std::string>& header,
      novatel_msgs::NovatelMessageHeader& msg)
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

  bool ParseNovatelTimeMessage(
      const NovatelSentence& sentence,
      novatel_msgs::TimePtr msg)
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

  bool ParseNovatelVelMessage(
      const NovatelSentence& sentence,
      novatel_msgs::NovatelVelocityPtr msg)
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

  bool parse_novatel_pos_msg(
      const NovatelSentence& sentence,
      novatel_msgs::NovatelPositionPtr msg)
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
      std::string& remaining,
      bool keep_nmea_container)
  {
    bool parse_error = false;
    size_t cur_idx = 0;

    const char NMEA_SENTENCE_FLAG = '$';
    const char NOVATEL_SENTENCE_FLAG = '#';

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
      else
      {
        // If for some reason we get here, we'll just iterate to the next message
        cur_idx = sentence_start + 1;
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
      std::vector<std::string>& vec,
      novatel_msgs::GpggaPtr msg)
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
      std::vector<std::string>& vec,
      novatel_msgs::GpgsaPtr msg)
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

  NmeaMessageParseResult parse_vectorized_gprmc_message(
      std::vector<std::string>& vec,
      novatel_msgs::GprmcPtr msg)
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
      const novatel_msgs::Gprmc& gprmc,
      const novatel_msgs::Gpgga& gpgga,
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

  bool ParseDouble(const std::string& string, double& value)
  {
    return swri_string_util::ToDouble(string, value) || string.empty();
  }

  bool ParseFloat(const std::string& string, float& value)
  {
    return swri_string_util::ToFloat(string, value) || string.empty();
  }

  bool ParseInt32(const std::string& string, int32_t& value, int32_t base)
  {
    return swri_string_util::ToInt32(string, value, base) || string.empty();
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
}
