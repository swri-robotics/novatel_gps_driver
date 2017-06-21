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
#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gpgsa.h>
#include <novatel_gps_msgs/Gpgsv.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelMessageHeader.h>
#include <novatel_gps_msgs/NovatelReceiverStatus.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>
#include <novatel_gps_msgs/Trackstat.h>

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
  const uint8_t NOVATEL_BINARY_SYNC_BYTE = 0xAA;

  const uint16_t BESTPOS_BINARY_MESSAGE_ID = 42;

  const size_t NOVATEL_MESSAGE_HEADER_LENGTH = 10;
  const size_t NOVATEL_OMNIHPPOS_BODY_LENGTH = 21;
  const size_t NOVATEL_POS_MSG_BODY_LENGTH = 21;
  const size_t NOVATEL_RANGE_BODY_FIELDS = 10;
  const size_t NOVATEL_TIME_BODY_FIELDS = 11;
  const size_t NOVATEL_TRACKSTAT_BODY_FIELDS = 4;
  const size_t NOVATEL_TRACKSTAT_CHANNEL_FIELDS = 10;
  const size_t NOVATEL_VEL_BODY_FIELDS = 8;

  enum NmeaMessageParseResult
  {
    ParseSucceededAndGpsDataValid,
    ParseSucceededAndGpsDataNotValid,
    ParseFailed
  };

  struct BinaryHeader
  {
    BinaryHeader() :
      sync0_(0xAA),
      sync1_(0x44),
      sync2_(0x12)
    {}
    uint8_t sync0_;
    uint8_t sync1_;
    uint8_t sync2_;
    uint8_t header_length_;
    uint16_t message_id_;
    int8_t message_type_;
    uint8_t port_address_;
    uint16_t message_length_;
    uint16_t sequence_;
    uint8_t idle_time_;
    uint8_t time_status_;
    uint16_t week_;
    uint32_t gpsec_;
    uint32_t receiver_status_;
    uint16_t reserved_;
    uint16_t receiver_sw_version_;
  };

  struct BinaryMessage
  {
    BinaryHeader header_;
    std::vector<uint8_t> data_;
    uint32_t crc_;
  };

  const std::string SOLUTION_STATUSES[] = {"SOL_COMPUTED", "INSUFFICIENT_OBS",
    "NO_CONVERGENCE", "SINGULARITY", "COV_TRACE", "TEST_DIST", "COLD_START",
    "V_H_LIMIT", "VARIANCE", "RESIDUALS", "RESERVED", "RESERVED", "RESERVED",
    "INTEGRITY_WARNING", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
    "PENDING", "INVALID_FIX", "RESERVED", "RESERVED", "INVALID_RATE"};
  const std::string POSITION_TYPES[] = { "NONE", "FIXEDPOS", "FIXEDHEIGHT", "RESERVED",
    "FLOATCONV", "WIDELANE", "NARROWLANE", "RESERVED", "DOPPLER_VELOCITY", "RESERVED",
    "RESERVED","RESERVED","RESERVED","RESERVED","RESERVED","RESERVED", "SINGLE",
    "PSRDIFF", "WAAS", "PROPOGATED", "OMNISTAR", "RESERVED", "RESERVED", "RESERVED",
    "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
    "RESERVED", "L1_FLOAT", "IONOFREE_FLOAT", "NARROW_FLOAT", "RESERVED", "RESERVED",
    "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
    "RESERVED", "RESERVED", "RESERVED", "RESERVED", "L1_INT", "WIDE_INT", "NARROW_INT",
    "RTK_DIRECT_INS", "INS_SBAS", "INS_PSRSP", "INS_PSRDIFF", "INS_RTKFLOAT", "INS_OMNISTAR",
    "INS_OMNISTAR_HP", "INS_OMNISTAR_XP", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
    "OMNISTAR_HP", "OMNISTAR_XP", "RESERVED", "RESERVED", "PPP_CONVERGING", "PPP",
    "RESERVED", "RESERVED", "RESERVED", "INS_PPP_CONVERGING", "INS_PPP" };
  const std::string DATUMS[] = { "BLANK",
    "ADIND", "ARC50", "ARC60", "AGD66", "AGD84", "BUKIT", "ASTRO", "CHATM", "CARTH", "CAPE",
    "DJAKA", "EGYPT", "ED50", "ED79", "GUNSG", "GEO49", "GRB36", "GUAM", "HAWAII", "KAUAI",
    "MAUI", "OAHU", "HERAT", "HJORS", "HONGK", "HUTZU", "INDIA", "IRE65", "KERTA", "KANDA",
    "LIBER", "LUZON", "MINDA", "MERCH", "NAHR", "NAD83", "CANADA", "ALASKA", "NAD27", "CARIBB",
    "MEXICO", "CAMER", "MINNA", "OMAN", "PUERTO", "QORNO", "ROME", "CHUA", "SAM56", "SAM69",
    "CAMPO", "SACOR", "YACAR", "TANAN", "TIMBA", "TOKYO", "TRIST", "VITI", "WAK60", "WGS72",
    "WGS84", "ZANDE", "USER", "CSRS", "ADIM", "ARSM", "ENW", "HTN", "INDB", "INDI",
    "IRL", "LUZA", "LUZB", "NAHC", "NASP", "OGBM", "OHAA", "OHAB", "OHAC", "OHAD",
    "OHIA", "OHIB", "OHIC", "OHID", "TIL", "TOYM" };

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
      novatel_gps_msgs::GpggaPtr msg);

  NmeaMessageParseResult parse_vectorized_gpgsa_message(
      std::vector<std::string>& vec,
      novatel_gps_msgs::GpgsaPtr msg);

  NmeaMessageParseResult ParseVectorizedGpgsvMessage(
      std::vector<std::string>& vec,
      novatel_gps_msgs::GpgsvPtr msg);

  NmeaMessageParseResult parse_vectorized_gprmc_message(
      std::vector<std::string>& vec,
      novatel_gps_msgs::GprmcPtr msg);

  void get_gps_fix_message(
      const novatel_gps_msgs::Gprmc& gprmc,
      const novatel_gps_msgs::Gpgga& gpgga,
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
      novatel_gps_msgs::NovatelReceiverStatus& receiver_status_msg);

  bool ParseNovatelBinaryHeader(
      const BinaryMessage& bin_msg,
      novatel_gps_msgs::NovatelMessageHeader& msg);

  bool parse_novatel_vectorized_header(
      const std::vector<std::string>& header,
      novatel_gps_msgs::NovatelMessageHeader& novatel_msg_header);

  bool ParseNovatelTimeMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::TimePtr time);

  bool ParseNovatelRangeMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::RangePtr msg);

  bool ParseNovatelTrackstatMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::TrackstatPtr msg);

  bool ParseNovatelVelMessage(
      const NovatelSentence& sentence,
      novatel_gps_msgs::NovatelVelocityPtr msg);

  bool parse_binary_novatel_pos_msg(
      const BinaryMessage& bin_msg,
      novatel_gps_msgs::NovatelPositionPtr ros_msg);

  bool parse_novatel_pos_msg(
      const NovatelSentence& sentence,
      novatel_gps_msgs::NovatelPositionPtr ros_msg);

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
      std::vector<BinaryMessage>& binary_messages,
      std::string& remaining,
      bool keep_nmea_container = false);

  double ParseDouble(const uint8_t* buffer);

  bool ParseDouble(const std::string& string, double& value);

  float ParseFloat(const uint8_t* buffer);

  bool ParseFloat(const std::string& string, float& value);

  int16_t ParseInt16(const uint8_t* buffer);

  bool ParseInt16(const std::string& string, int16_t& value, int32_t base = 10);

  int32_t ParseInt32(const uint8_t* buffer);

  bool ParseInt32(const std::string& string, int32_t& value, int32_t base = 10);

  bool ParseUInt8(const std::string& string, uint8_t& value, int32_t base = 10);

  uint16_t ParseUInt16(const uint8_t* buffer);

  bool ParseUInt16(const std::string& string, uint16_t& value, int32_t base = 10);

  uint32_t ParseUInt32(const uint8_t* buffer);

  bool ParseUInt32(const std::string& string, uint32_t& value, int32_t base = 10);
}

#endif  // NOVTEL_OEM628_NOVATEL_MESSAGE_PARSER_H_
