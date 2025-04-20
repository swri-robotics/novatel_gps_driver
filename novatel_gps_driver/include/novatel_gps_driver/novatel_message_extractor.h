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

#ifndef NOVTEL_OEM628_NOVATEL_MESSAGE_PARSER_H_
#define NOVTEL_OEM628_NOVATEL_MESSAGE_PARSER_H_

#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gps_msgs/msg/gps_fix.hpp>
#include <novatel_gps_msgs/msg/gpgga.hpp>
#include <novatel_gps_msgs/msg/gpgsa.hpp>
#include <novatel_gps_msgs/msg/gpgsv.hpp>
#include <novatel_gps_msgs/msg/gprmc.hpp>
#include <novatel_gps_msgs/msg/novatel_corrected_imu_data.hpp>
#include <novatel_gps_msgs/msg/novatel_position.hpp>
#include <novatel_gps_msgs/msg/novatel_message_header.hpp>
#include <novatel_gps_msgs/msg/novatel_receiver_status.hpp>
#include <novatel_gps_msgs/msg/novatel_velocity.hpp>
#include <novatel_gps_msgs/msg/range.hpp>
#include <novatel_gps_msgs/msg/time.hpp>
#include <novatel_gps_msgs/msg/trackstat.hpp>

#include <novatel_gps_driver/binary_message.h>
#include <novatel_gps_driver/nmea_sentence.h>
#include <novatel_gps_driver/novatel_sentence.h>

#include <rclcpp/logger.hpp>

namespace novatel_gps_driver
{
  class NovatelMessageExtractor
  {
  public:
    explicit NovatelMessageExtractor(rclcpp::Logger logger);
    /**
      * @brief Extracts messages from a buffer of NovAtel data.
      *
      * This will search through the "input" string for any valid NMEA or NovAtel ASCII
      * messages as well as any binary NovAtel messages, place them into the provided
      * containers, and also provide any leftover bytes at the end of the buffer that
      * were not part of a valid sentence.
      *
      * @param[in] input A buffer of data to search for messages
      * @param[out] nmea_sentences NMEA sentences found in the buffer
      * @param[out] novatel_sentences ASCII NovAtel sentences found in the buffer
      * @param[out] binary_messages Binary NovAtel message found in the buffer
      * @param[out] remaining Any bytes left after the last complete sentence
      * in the buffer
      * @param[in] keep_nmea_container "true" to preserve message begin & end markers
      * around parsed NMEA sentences
      * @return false if there were any errors parsing sentences
      */
    bool ExtractCompleteMessages(
        const std::string& input,
        std::vector<NmeaSentence>& nmea_sentences,
        std::vector<NovatelSentence>& novatel_sentences,
        std::vector<BinaryMessage>& binary_messages,
        std::string& remaining,
        bool keep_nmea_container = false);

    /**
     * @brief Iterates through the provided messages to find the first GPGGA
     * or GPRMC message with a valid UTC time, then returns it.
     * @param[in] sentences A list of NMEA sentences to search for UTC times.
     * @return The UTC time in seconds, if found; 0 if not found.
     */
    double GetMostRecentUtcTime(const std::vector<NmeaSentence>& sentences);

    /**
     * @brief Combines data receives in GPRMC and GPGGA message to produce
     * a gps_msgs/GPSFixPtr ROS message.
     * @param[in] gprmc A valid GPRMC message.
     * @param[in] gpgga A valid GPGGA message.
     * @param[out] gps_fix An initialised GPSFixPtr message must be provided;
     * it will be filled in based on the provided GPRMC/GPGGA messages.
     */
    void GetGpsFixMessage(
        const novatel_gps_msgs::msg::Gprmc& gprmc,
        const novatel_gps_msgs::msg::Gpgga& gpgga,
        const gps_msgs::msg::GPSFix::UniquePtr& gps_fix);

  private:
    // Constants for parsing message structures
    /// Precedes checkums in ASCII messages
    static const std::string CHECKSUM_FLAG;
    /// Separates data fields in ASCII messages
    static const std::string FIELD_SEPARATOR;
    /// Separates header from body in ASCII NovAtel messages
    static const std::string HEADER_SEPARATOR;
    /// Indicates the beginning of a NMEA sentence
    static const std::string NMEA_SENTENCE_FLAG;
    /// Indicates the beginning of an ASCII NovAtel message
    static const std::string NOVATEL_SENTENCE_FLAG;
    /// Used to search for both types of ASCII messages
    static const std::string NOVATEL_ASCII_FLAGS;
    /// Indicates the beginning of a binary NovAtel message
    static const std::string NOVATEL_BINARY_SYNC_BYTES;
    static const std::string NOVATEL_BINARY_SYNC_BYTES2;
    /// Indicates the end of an ASCII message
    static const std::string NOVATEL_ENDLINE;

    static constexpr uint32_t NOVATEL_CRC32_POLYNOMIAL = 0xEDB88320L;

    rclcpp::Logger logger_;

    // From Novatel OEMV® Family Firmware Reference Manual
    /**
     * @brief Calculates the CRC-32 of a block of data all at once
     * @param ulCount Number of bytes in the data block
     * @param ucBuffer Data block
     * @return CRC-32 of the data block
     */
    uint32_t CalculateBlockCRC32(
        uint32_t ulCount,
        const uint8_t* ucBuffer);

    /**
     * @brief Calculates the CRC-32 of a single value
     * @param i The value to process
     * @return That value's CRC-32
     */
    uint32_t CRC32Value(int32_t i);


    /**
     * @brief Searches for a valid ASCII message within a string.
     *
     * What constitutes a valid ASCII message is defined by:
     * http://docs.novatel.com/OEM7/Content/Messages/ASCII.htm
     *
     * We check whether a substring:
     * 1) Starts with '#' or '$'
     * 2) Ends with "\r\n"
     * 3) Between those values, only contains characters with an ASCII value
     *    of 9, 10, 11, 13, or between 32 and 126 (inclusive)
     *
     * @param[in] sentence The string to search within
     * @param[in] current_idx The position in the string to search from
     * @param[out] start_idx The position of the earliest '#' or '$', if found
     * @param[out] end_idx The position of the first '\r' after the start, if "\r\n" was found
     * @param[out] invalid_char_idx If an invalid character was found between the
     *        start and end, its index
     */
    void FindAsciiSentence(const std::string& sentence,
                           size_t current_idx,
                           size_t& start_idx,
                           size_t& end_idx,
                           size_t& invalid_char_idx);

    /**
     * @brief Extracts a binary message from a data buffer.
     * @param[in] str The data buffer to search through.
     * @param[in] start_idx An index in the buffer to begin searching from.
     * @param[out] msg The parsed message.
     * @return >0: the number of bytes parsed into the message
     *         -1: Not enough data was available to parse the whole message
     *         -2: The message's checksum was invalid
     */
    int32_t GetBinaryMessage(const std::string& str,
                             size_t start_idx,
                             BinaryMessage& msg);

    /**
     * @brief Splits an ASCII NovAtel message up into header and body parts.
     * @param[in] sentence The NovAtel message to parse.
     * @param[out] message_id The message's ID.
     * @param[out] header All of the fields in the message's header.
     * @param[out] body All of the fields in the message's body.
     * @return false if there were errors parsing the message, true otherwise.
     */
    bool GetNovatelMessageParts(
        const std::string& sentence,
        std::string& message_id,
        std::vector<std::string>& header,
        std::vector<std::string>& body);

    /**
     * @brief Extracts an NMEA sentence from an input string.
     * @param[in] str The string to extract an NMEA sentence from.
     * @param[in] start_idx The index to begin searching at.
     * @param[out] sentence If a valid sentence was found, it will be placed here.
     * @param[out] keep_container "true" to keep sentence & checksum position marks
     * in the output sentence.
     * @return 0: Success
     *         -1: Not enough data to parse a complete sentence
     *         1: Sentence's checksum was invalid
     */
    int32_t GetNmeaSentence(
        const std::string& str,
        size_t start_idx,
        std::string& sentence,
        bool keep_container = false);

    /**
     * @brief Extracts a NovAtel sentence from an input string.
     * @param[in] str The string to search for a NovAtel sentence.
     * @param[in] start_idx The index to begin searching from.
     * @param[out] sentence The extracted sentence.
     * @return 0: Success
     *         -1: Not eenough data to parse a complete sentence
     *         1: Checksum was invalid
     */
    int32_t GetNovatelSentence(
        const std::string& str,
        size_t start_idx,
        std::string& sentence);

    /**
     * @brief Finds the location of the next checksum in a valid ASCII sentence.
     * @param str A buffer to search for a checksum.
     * @param start_idx The location to begin searching from.
     * @return The index of that sentence's checksum.
     */
    size_t GetSentenceChecksumStart(
        const std::string& str,
        size_t start_idx);

    /**
     * @brief Calculates the checksum of a NMEA sentence.
     * @param sentence The sentence to check.
     * @return That sentence's checksum.
     */
    uint8_t NmeaChecksum(const std::string& sentence);

    /**
     * @brief Takes a sentence extracted by GetNovatelSentence and converts it
     * into a data structure.
     * @param[in] data A valid, extracted ASCII NovAtel sentence.
     * @param[out] sentence A data structure containing the sentence.
     * @return false if it failed to parse the sentence, true otherwise.
     */
    bool VectorizeNovatelSentence(
        const std::string& data,
        NovatelSentence& sentence);

    /**
     * @brief Takes a sentence extracted by GetNmeaSentence and converts it
     * into a data structure.
     * @param[in] data A valid, extracted ASCII NMEA sentence.
     * @param[out] sentence A data structure containing the sentence.
     * @return false if it failed to parse the sentence, true otherwise.
     */
    void VectorizeNmeaSentence(
        const std::string& sentence,
        NmeaSentence& vectorized_message);

    /**
     * Splits a string into a vector using any of the provided characters
     * as delimiters.
     * @param[in] str The string to parse.
     * @param[out] vectorized_message The split results.
     * @param[in] delimiters A set of characters to use as delimiters.
     */
    void VectorizeString(
        const std::string& str,
        std::vector<std::string>& vectorized_message,
        const std::string& delimiters);
  };
}

#endif  // NOVATEL_GPS_DRIVER_NOVATEL_MESSAGE_PARSER_H_
