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

#ifndef NOVTEL_OEM628_NOVATEL_MESSAGE_PARSER_H_
#define NOVTEL_OEM628_NOVATEL_MESSAGE_PARSER_H_

#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gps_common/GPSFix.h>
#include <novatel_msgs/Gpgga.h>
#include <novatel_msgs/Gpgsa.h>
#include <novatel_msgs/Gprmc.h>
#include <novatel_msgs/NovatelPosition.h>
#include <novatel_msgs/NovatelMessageHeader.h>
#include <novatel_msgs/NovatelReceiverStatus.h>
#include <novatel_msgs/NovatelVelocity.h>
#include <novatel_msgs/Time.h>

#define NOVATEL_CRC32_POLYNOMIAL       0xEDB88320L

#ifndef KNOTS_TO_MPS
#define KNOTS_TO_MPS    0.5144444
#endif

#define ADDRESS_FIELD_MAX_LENGTH        10
#define NMEA_SEQUENCE_MAX_LENGTH        81

#define ERR_INIT_HIGH                   1e20;

namespace novatel_oem628
{
  const char NMEA_SENTENCE_FLAG = '$';
  const char NOVATEL_SENTENCE_FLAG = '#';

  const size_t NOVATEL_MESSAGE_HEADER_LENGTH = 10;
  const size_t NOVATEL_OMNIHPPOS_BODY_LENGTH = 21;
  const size_t NOVATEL_POS_MSG_BODY_LENGTH = 21;
  const size_t NOVATEL_TIME_BODY_FIELDS = 11;
  const size_t NOVATEL_VEL_BODY_FIELDS = 8;

  enum NmeaMessageParseResult
  {
    ParseSucceededAndGpsDataValid,
    ParseSucceededAndGpsDataNotValid,
    ParseFailed
  };

  struct NovatelSentence
  {
    std::string id;
    std::vector<std::string> header;
    std::vector<std::string> body;
  };

  struct NmeaSentence
  {
    std::string id;
    std::vector<std::string> body;
  };

  struct NovatelTime
  {
    uint32_t clock_status;
    double offset;
    double offset_std;
    double utc_offset;
    uint32_t utc_year;
    uint32_t utc_month;
    uint32_t utc_day;
    uint32_t utc_hour;
    uint32_t utc_min;
    uint32_t utc_ms;
    uint32_t utc_status;
  };

  double GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences);

  NmeaMessageParseResult parse_vectorized_gpgga_message(
      std::vector<std::string>& vec,
      novatel_msgs::GpggaPtr msg);

  NmeaMessageParseResult parse_vectorized_gpgsa_message(
      std::vector<std::string>& vec,
      novatel_msgs::GpgsaPtr msg);

  NmeaMessageParseResult parse_vectorized_gprmc_message(
      std::vector<std::string>& vec,
      novatel_msgs::GprmcPtr msg);

  void get_gps_fix_message(
      const novatel_msgs::Gprmc& gprmc,
      const novatel_msgs::Gpgga& gpgga,
      gps_common::GPSFixPtr gps_fix);

  double UtcFloatToSeconds(double utc_float);

  // From Novatel OEMV® Family Firmware Reference Manual
  /* --------------------------------------------------------------------------
  Calculate a CRC value to be used by CRC calculation functions.
  -------------------------------------------------------------------------- */
  uint32_t CRC32Value(int32_t i);

  /* --------------------------------------------------------------------------
  Calculates the CRC-32 of a block of data all at once
  -------------------------------------------------------------------------- */
  uint32_t CalculateBlockCRC32(
      uint32_t ulCount,
      /* Number of bytes in the data block */
      const uint8_t* ucBuffer ); /* Data block */


  uint8_t NmeaChecksum(const std::string& sentence);

  size_t get_next_sentence_start(
      const std::string& str,
      size_t start_idx);

  size_t get_sentence_checksum_start(
      const std::string& str,
      size_t start_idx);

  void vectorize_string(
      const std::string& str,
      std::vector<std::string>& vectorized_message,
      char delimiter);

  bool get_novatel_message_parts(
      const std::string& sentence,
      std::string& message_id,
      std::vector<std::string>& header,
      std::vector<std::string>& body);

  void get_novatel_receiver_status_msg(
      uint32_t status,
      novatel_msgs::NovatelReceiverStatus& receiver_status_msg);

  bool parse_novatel_vectorized_header(
      const std::vector<std::string>& header,
      novatel_msgs::NovatelMessageHeader& novatel_msg_header);

  bool ParseNovatelTimeMessage(
      const NovatelSentence& sentence,
      novatel_msgs::TimePtr time);

  bool ParseNovatelVelMessage(
      const NovatelSentence& sentence,
      novatel_msgs::NovatelVelocityPtr msg);

  bool parse_novatel_pos_msg(
      const NovatelSentence& sentence,
      novatel_msgs::NovatelPositionPtr ros_msg);

  /**
   * @brief      Gets the novatel message sentence
   */
  int32_t get_novatel_sentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence);

  int32_t get_nmea_sentence(
      const std::string& str,
      size_t start_idx,
      std::string& sentence,
      bool keep_container = false);

  bool VectorizeNovatelSentence(
      const std::string& data,
      NovatelSentence& sentence);

  void VectorizeNmeaSentence(
      const std::string& sentence,
      NmeaSentence& vectorized_message);

  /**
   * Returns false if a parse error is encountered.
   */
  bool extract_complete_sentences(
      const std::string input,
      std::vector<NmeaSentence>& nmea_sentences,
      std::vector<NovatelSentence>& novatel_sentences,
      std::string& remaining,
      bool keep_nmea_container = false);

  bool ParseDouble(const std::string& string, double& value);

  bool ParseFloat(const std::string& string, float& value);

  bool ParseInt32(const std::string& string, int32_t& value, int32_t base = 10);

  bool ParseUInt32(const std::string& string, uint32_t& value, int32_t base = 10);

  bool ParseUInt8(const std::string& string, uint8_t& value, int32_t base = 10);
}

#endif  // NOVTEL_OEM628_NOVATEL_MESSAGE_PARSER_H_
