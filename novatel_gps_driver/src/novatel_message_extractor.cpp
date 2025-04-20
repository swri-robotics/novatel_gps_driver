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

#include <novatel_gps_driver/novatel_message_extractor.h>

#include <limits>
#include <sstream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <novatel_gps_driver/parsers/header.h>
#include <novatel_gps_driver/parsers/gpgga.h>
#include <novatel_gps_driver/parsers/gprmc.h>

#include <rclcpp/logging.hpp>

namespace novatel_gps_driver
{
  const std::string NovatelMessageExtractor::CHECKSUM_FLAG = "*";
  const std::string NovatelMessageExtractor::FIELD_SEPARATOR = ",";
  const std::string NovatelMessageExtractor::HEADER_SEPARATOR = ";";
  const std::string NovatelMessageExtractor::NMEA_SENTENCE_FLAG = "$";
  const std::string NovatelMessageExtractor::NOVATEL_SENTENCE_FLAG = "#";
  const std::string NovatelMessageExtractor::NOVATEL_ASCII_FLAGS = "$#";
  const std::string NovatelMessageExtractor::NOVATEL_BINARY_SYNC_BYTES = "\xAA\x44\x12";
  const std::string NovatelMessageExtractor::NOVATEL_BINARY_SYNC_BYTES2 = "\xAA\x44\x13";
  const std::string NovatelMessageExtractor::NOVATEL_ENDLINE = "\r\n";
  
  NovatelMessageExtractor::NovatelMessageExtractor(rclcpp::Logger logger) :
    logger_(logger)
  {}
  
  uint32_t NovatelMessageExtractor::CRC32Value(int32_t i)
  {
    int32_t j;
    uint32_t ulCRC;
    ulCRC = static_cast<uint32_t>(i);
    for ( j = 8 ; j > 0; j-- )
    {
      if ( ulCRC & 1u )
        ulCRC = static_cast<uint32_t>(( ulCRC >> 1u ) ^ NOVATEL_CRC32_POLYNOMIAL);
      else
        ulCRC >>= 1u;
    }
    return ulCRC;
  }

  uint32_t NovatelMessageExtractor::CalculateBlockCRC32(
      uint32_t ulCount,          // Number of bytes in the data block
      const uint8_t* ucBuffer )  // Data block
  {
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;
    while ( ulCount-- != 0 )
    {
      ulTemp1 = static_cast<uint32_t>(( ulCRC >> 8u ) & 0x00FFFFFFL);
      ulTemp2 = CRC32Value( ((int32_t) ulCRC ^ *ucBuffer++ ) & 0xffu );
      ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
  }

  uint8_t NovatelMessageExtractor::NmeaChecksum(const std::string& sentence)
  {
    uint8_t checksum = 0;
    std::string::const_iterator it = sentence.begin();
    for (; it != sentence.end(); ++it)
    {
      checksum ^= *it;
    }
    return checksum;
  }

  size_t NovatelMessageExtractor::GetSentenceChecksumStart(const std::string& str, size_t start_idx)
  {
    return str.find(CHECKSUM_FLAG, start_idx);
  }

  void NovatelMessageExtractor::VectorizeString(
      const std::string& str,
      std::vector<std::string>& vectorized_message,
      const std::string& delimiters)
  {
    boost::algorithm::split(vectorized_message, str, boost::algorithm::is_any_of(delimiters));
  }

  bool NovatelMessageExtractor::GetNovatelMessageParts(
      const std::string& sentence,
      std::string& message_id,
      std::vector<std::string>& header,
      std::vector<std::string>& body)
  {
    message_id.clear();
    header.clear();
    body.clear();

    std::vector<std::string> vectorized_message;
    VectorizeString(sentence, vectorized_message, HEADER_SEPARATOR);

    if (vectorized_message.size() != 2)
    {
      return false;
    }

    VectorizeString(vectorized_message[0], header, FIELD_SEPARATOR);

    VectorizeString(vectorized_message[1], body, FIELD_SEPARATOR);

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

  int32_t NovatelMessageExtractor::GetBinaryMessage(const std::string& str,
                           size_t start_idx,
                           BinaryMessage& msg)
  {
    if (str.length() < HeaderParser::BINARY_HEADER_LENGTH + 4)
    {
      // The shortest a binary message can be (header + no data + CRC)
      // is 32 bytes, so just return if we don't have at least that many.
      RCLCPP_DEBUG(this->logger_, "Binary message was too short to parse.");
      return -1;
    }

    RCLCPP_DEBUG(this->logger_, "Reading binary header.");
    if (str[start_idx+2] == static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[2])){
      msg.header_.ParseHeader(reinterpret_cast<const uint8_t*>(&str[start_idx]));
    }
    else if (str[start_idx+2] == static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES2[2]))
    {
      msg.header_.ParseShortHeader(reinterpret_cast<const uint8_t*>(&str[start_idx]));
    }
    else  {
      RCLCPP_ERROR(this->logger_, "Sync bytes were incorrect; this should never happen and is definitely a bug: %x",
        str[start_idx+2]);
      return -2;
    }
    auto data_start = static_cast<uint16_t>(msg.header_.header_length_ + start_idx);
    uint16_t data_length = msg.header_.message_length_;

    if (msg.header_.sync0_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[0]) ||
        msg.header_.sync1_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[1]) ||
        (msg.header_.sync2_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES[2]) &&
        msg.header_.sync2_ != static_cast<uint8_t>(NOVATEL_BINARY_SYNC_BYTES2[2]))
      )
    {
      RCLCPP_ERROR(this->logger_, "Sync bytes were incorrect; this should never happen and is definitely a bug: %x %x %x",
               msg.header_.sync0_, msg.header_.sync1_, msg.header_.sync2_);
      return -2;
    }

    if (msg.header_.header_length_ != HeaderParser::BINARY_HEADER_LENGTH        ||
        msg.header_.header_length_ != HeaderParser::BINARY_SHORT_HEADER_LENGTH
    )
    {
      RCLCPP_DEBUG(this->logger_, "Binary header length was unexpected: %u (expected %u, or %u)",
               msg.header_.header_length_, HeaderParser::BINARY_HEADER_LENGTH, HeaderParser::BINARY_SHORT_HEADER_LENGTH);
    }

    RCLCPP_DEBUG(this->logger_, "Msg ID: %u    Data start / length: %u / %u",
              msg.header_.message_id_, data_start, data_length);

    if (data_start + data_length + 4 > str.length())
    {
      RCLCPP_DEBUG(this->logger_, "Not enough data.");
      return -1;
    }

    RCLCPP_DEBUG(this->logger_, "Reading binary message data.");
    msg.data_.resize(data_length);
    std::copy(&str[data_start], &str[data_start+data_length], reinterpret_cast<char*>(&msg.data_[0]));

    RCLCPP_DEBUG(this->logger_, "Calculating CRC.");

    uint32_t crc = CalculateBlockCRC32(static_cast<uint32_t>(msg.header_.header_length_) + data_length,
                                       reinterpret_cast<const uint8_t*>(&str[start_idx]));

    RCLCPP_DEBUG(this->logger_, "Reading CRC.");
    msg.crc_ = ParseUInt32(reinterpret_cast<const uint8_t*>(&str[data_start+data_length]));

    if (crc != msg.crc_)
    {
      // Invalid CRC
      RCLCPP_DEBUG(this->logger_, "Invalid CRC;  Calc: %u    In msg: %u", crc, msg.crc_);
      return -2;
    }

    // On success, return how many bytes we read

    RCLCPP_DEBUG(this->logger_, "Finishing reading binary message.");
    return static_cast<int32_t>(msg.header_.header_length_ + data_length + 4);
  }

  int32_t NovatelMessageExtractor::GetNovatelSentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence)
  {
    sentence.clear();

    size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
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
      uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
      uint64_t calculated_checksum = CalculateBlockCRC32(
          static_cast<uint32_t>(sentence.size()),
          reinterpret_cast<const uint8_t*>(sentence.c_str()));

      if (checksum == ULONG_MAX)
      {
        // Invalid checksum -- strtoul failed
        return 1;
      }
      else if(static_cast<uint32_t>(checksum) == calculated_checksum)
      {
        return 0;
      }
      else
      {
        RCLCPP_WARN(this->logger_, "Expected checksum: [%lx]", calculated_checksum);
        // Invalid checksum
        return 1;
      }
    }
  }

  int32_t NovatelMessageExtractor::GetNmeaSentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence,
      bool keep_container)
  {
    sentence.clear();

    size_t checksum_start = GetSentenceChecksumStart(str, start_idx);
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
      uint64_t checksum = std::strtoul(checksum_str.c_str(), nullptr, 16);
      uint64_t calculated_checksum = NmeaChecksum(sentence);
      if (checksum == ULONG_MAX)
      {
        // Invalid checksum
        return 1;
      }
      else if(static_cast<uint32_t>(checksum) == calculated_checksum)
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
        RCLCPP_WARN(this->logger_, "Expected: [%lx]", calculated_checksum);
        // Invalid checksum
        return 1;
      }
    }
  }

  void NovatelMessageExtractor::FindAsciiSentence(const std::string& sentence,
                         size_t current_idx,
                         size_t& start_idx,
                         size_t& end_idx,
                         size_t& invalid_char_idx)
  {
    start_idx = sentence.find_first_of(NOVATEL_ASCII_FLAGS, current_idx);
    end_idx = std::string::npos;
    invalid_char_idx = std::string::npos;

    if (start_idx == std::string::npos)
    {
      return;
    }

    end_idx = sentence.find(NOVATEL_ENDLINE, start_idx);

    size_t search_stop_idx = std::min(end_idx, sentence.length());
    for (size_t i = start_idx; i < search_stop_idx; i++)
    {
      if (sentence[i] == 9 || sentence[i] == 10 || sentence[i] == 11 || sentence[i] == 13 ||
          (sentence[i] >= 32 && sentence[i] <= 126))
      {
        continue;
      }

      invalid_char_idx = i;
      break;
    }
  }

  bool NovatelMessageExtractor::VectorizeNovatelSentence(
      const std::string& data,
      NovatelSentence& sentence)
  {
    return GetNovatelMessageParts(
        data, sentence.id, sentence.header, sentence.body);
  }

  void NovatelMessageExtractor::VectorizeNmeaSentence(
    const std::string& sentence,
    NmeaSentence& vectorized_message)
  {
    VectorizeString(sentence, vectorized_message.body, FIELD_SEPARATOR);
    if (!vectorized_message.body.empty())
    {
      vectorized_message.id = vectorized_message.body.front();
    }
  }

  bool NovatelMessageExtractor::ExtractCompleteMessages(
      const std::string& input,
      std::vector<NmeaSentence>& nmea_sentences,
      std::vector<NovatelSentence>& novatel_sentences,
      std::vector<BinaryMessage>& binary_messages,
      std::string& remaining,
      bool keep_nmea_container)
  {
    bool parse_error = false;

    size_t sentence_start = 0;
    while(sentence_start != std::string::npos && sentence_start < input.size())
    {
      size_t ascii_start_idx;
      size_t ascii_end_idx;
      size_t invalid_ascii_idx;
      size_t binary_start_idx = input.find(NOVATEL_BINARY_SYNC_BYTES.substr(0, 2), sentence_start);

      FindAsciiSentence(input, sentence_start, ascii_start_idx, ascii_end_idx, invalid_ascii_idx);

      RCLCPP_DEBUG(this->logger_, "Binary start: %lu   ASCII start / end / invalid: %lu / %lu / %lu",
                binary_start_idx, ascii_start_idx, ascii_end_idx, invalid_ascii_idx);

      if (binary_start_idx == std::string::npos && ascii_start_idx == std::string::npos)
      {
        // If we don't see either a binary or an ASCII message, just give up.
        break;
      }

      if (ascii_start_idx == std::string::npos ||
          (binary_start_idx != std::string::npos && binary_start_idx < ascii_start_idx))
      {
        // If we see a binary header or if there's both a binary and ASCII header but
        // the binary one comes first, try to parse it.
        BinaryMessage cur_msg;
        int32_t result = GetBinaryMessage(input, binary_start_idx, cur_msg);
        if (result > 0)
        {
          binary_messages.push_back(cur_msg);
          sentence_start += binary_start_idx + result;
          RCLCPP_DEBUG(this->logger_, "Parsed a binary message with %u bytes.", result);
        }
        else if (result == -1)
        {
          // Sentence is not complete, add it to the remaining and break;
          remaining = input.substr(binary_start_idx);
          RCLCPP_DEBUG(this->logger_, "Binary message was incomplete, waiting for more.");
          break;
        }
        else
        {
          // Sentence had an invalid checksum, just iterate to the next sentence
          sentence_start += 1;
          RCLCPP_WARN(this->logger_, "Invalid binary message checksum");
          parse_error = true;
        }
      }
      else
      {
        // If we saw the beginning of an ASCII message, try to parse it.
        size_t ascii_len = ascii_end_idx - ascii_start_idx;
        if (invalid_ascii_idx != std::string::npos)
        {
          // If we see an invalid character, don't even bother trying to parse
          // the rest of the message.  By this point we also know there's no
          // binary header before this point, so just skip the data.
          RCLCPP_WARN(this->logger_, "Invalid ASCII char: [%s]", input.substr(ascii_start_idx, ascii_len).c_str());
          RCLCPP_WARN(this->logger_, "                     %s^", std::string(invalid_ascii_idx-ascii_start_idx-1, ' ').c_str());
          sentence_start += invalid_ascii_idx + 1;
        }
        else if (ascii_end_idx != std::string::npos)
        {
          // If we've got a start, an end, and no invalid characters, we've
          // got a valid ASCII message.
          RCLCPP_DEBUG(this->logger_, "ASCII sentence:\n[%s]", input.substr(ascii_start_idx, ascii_len).c_str());
          if (input[ascii_start_idx] == NMEA_SENTENCE_FLAG[0])
          {
            std::string cur_sentence;
            int32_t result = GetNmeaSentence(
                input,
                ascii_start_idx,
                cur_sentence,
                keep_nmea_container);
            if (result == 0)
            {
              nmea_sentences.emplace_back(NmeaSentence());
              VectorizeNmeaSentence(cur_sentence, nmea_sentences.back());
              sentence_start = ascii_end_idx;
            }
            else if (result < 0)
            {
              // Sentence is not complete, add it to the remaining and break.
              // This is legacy code from before FindAsciiSentence was implemented,
              // and it will probably never happen, but it doesn't hurt anything to
              // have it here.
              remaining = input.substr(ascii_start_idx);
              RCLCPP_DEBUG(this->logger_, "Waiting for more NMEA data.");
              break;
            }
            else
            {
              RCLCPP_WARN(this->logger_, "Invalid NMEA checksum for: [%s]",
                       input.substr(ascii_start_idx, ascii_len).c_str());
              // Sentence had an invalid checksum, just iterate to the next sentence
              sentence_start += 1;
              parse_error = true;
            }
          }
          else if (input[ascii_start_idx] == NOVATEL_SENTENCE_FLAG[0])
          {
            std::string cur_sentence;
            int32_t result = GetNovatelSentence(input, ascii_start_idx, cur_sentence);
            if (result == 0)
            {
              // Send to parser for testing:
              novatel_sentences.emplace_back(NovatelSentence());
              if (!VectorizeNovatelSentence(cur_sentence, novatel_sentences.back()))
              {
                novatel_sentences.pop_back();
                parse_error = true;
                // TODO pjr Replace with _THROTTLE when it's available
                RCLCPP_ERROR(this->logger_, "Unable to vectorize novatel sentence");
              }
              sentence_start = ascii_end_idx;
            }
            else if (result < 0)
            {

              // Sentence is not complete, add it to the remaining and break.
              // This is legacy code from before FindAsciiSentence was implemented,
              // and it will probably never happen, but it doesn't hurt anything to
              // have it here.
              remaining = input.substr(ascii_start_idx);
              RCLCPP_DEBUG(this->logger_, "Waiting for more NovAtel data.");
              break;
            }
            else
            {
              // Sentence had an invalid checksum, just iterate to the next sentence
              sentence_start += 1;
              RCLCPP_WARN(this->logger_, "Invalid NovAtel checksum for: [%s]",
                       input.substr(ascii_start_idx, ascii_len).c_str());
              parse_error = true;
            }
          }
        }
        else
        {
          RCLCPP_DEBUG(this->logger_, "Incomplete ASCII sentence, waiting for more.");
          remaining = input.substr(ascii_start_idx);
          break;
        }
      }
    }

    return !parse_error;
  }

  double NovatelMessageExtractor::GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences)
  {
    std::vector<NmeaSentence>::const_reverse_iterator iter;
    for (iter = sentences.rbegin(); iter != sentences.rend(); iter++)
    {
      if (iter->id == GpggaParser::MESSAGE_NAME || iter->id == GprmcParser::MESSAGE_NAME)
      {
        if (iter->body.size() > 1)
        {
          if (iter->body[1].empty() || iter->body[1] == "0")
          {
            return 0;
          }
          else
          {
            try
            {
              return boost::lexical_cast<double>(iter->body[1]);
            }
            catch (boost::bad_lexical_cast& e)
            {
              return 0;
            }
          }
        }
      }
    }

    return 0;
  }

  void NovatelMessageExtractor::GetGpsFixMessage(
      const novatel_gps_msgs::msg::Gprmc& gprmc,
      const novatel_gps_msgs::msg::Gpgga& gpgga,
      const gps_msgs::msg::GPSFix::UniquePtr& gps_fix)
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

    switch (gpgga.gps_qual)
    {
      case novatel_gps_msgs::msg::Gpgga::GPS_QUAL_INVALID:
        gps_fix->status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
        break;
      case novatel_gps_msgs::msg::Gpgga::GPS_QUAL_WASS:
        gps_fix->status.status = gps_msgs::msg::GPSStatus::STATUS_WAAS_FIX;
        break;
      default:
        // Other statuses don't seem to have an exact equivalent, so map them all to STATUS_FIX
        gps_fix->status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;
        break;
    }
    gps_fix->status.satellites_used = static_cast<uint16_t>(gpgga.num_sats);

  }

}
