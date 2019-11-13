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

#ifndef NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H
#define NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H

#include <novatel_gps_driver/binary_message.h>
#include <novatel_gps_driver/nmea_sentence.h>
#include <novatel_gps_driver/novatel_sentence.h>

#include <novatel_gps_driver/parsers/parsing_utils.h>
#include <novatel_gps_driver/parsers/parse_exception.h>

#include <cstdint>

namespace novatel_gps_driver
{
  /**
   * Base class for converting extracted NMEA and NovAtel sentences into ROS
   * messages
   *
   * Subclasses that parse NMEA messages should implement
   * ParseAscii(const NmeaSentence&); subclasses that parse NovAtel messages
   * should implement both ParseAscii(const NovatelSentence&) and
   * ParseBinary(const BinaryMessage&).
   *
   * For documentation on exact message structures, see:
   * http://docs.novatel.com/OEM7/Content/Logs/Core_Logs.htm
   *
   * @tparam T The ROS message Ptr type that the parser should produce.
   * UniquePtr types are preferred for efficient intraprocess communication,
   * but SharedPtrs may be necessary if the driver needs multiple references
   * to a message for synchronization or other purposes.
   */
  template<typename T>
  class MessageParser
  {
  public:
    virtual ~MessageParser() = default;

    typedef T MessageType;

    /**
     * @return The binary message ID. Should be 0 for messages that have no
     * binary representation.
     */
    virtual uint32_t GetMessageId() const = 0;
    /**
     * @return The ASCII message name.
     */
    virtual const std::string GetMessageName() const = 0;

    /**
     * @brief Converts bin_msg into a ROS message pointer and returns it.
     *
     * The returned value must not be NULL.  ParseException should be thrown
     * if there are any issues parsing the message.
     * @param[in] bin_msg The message to convert.
     * @return A valid ROS message pointer.
     */
    virtual T ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
    {
      throw ParseException("ParseBinary not implemented.");
    };

    /**
     * @brief Converts sentence into a ROS message pointer and returns it.
     *
     * The returned value must not be NULL.  ParseException should be thrown
     * if there are any issues parsing the message.
     * @param[in] bin_msg The message to convert.
     * @return A valid ROS message pointer.
     */
    virtual T ParseAscii(const NovatelSentence& sentence) noexcept(false)
    {
      throw ParseException("ParseAscii not implemented.");
    };

    /**
     * @brief Converts sentence into a ROS message pointer and returns it.
     *
     * The returned value must not be NULL.  ParseException should be thrown
     * if there are any issues parsing the message.
     * @param[in] bin_msg The message to convert.
     * @return A valid ROS message pointer.
     */
    virtual T ParseAscii(const NmeaSentence& sentence) noexcept(false)
    {
      throw ParseException("ParseAscii not implemented.");
    };
  };
}

#endif //NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H
