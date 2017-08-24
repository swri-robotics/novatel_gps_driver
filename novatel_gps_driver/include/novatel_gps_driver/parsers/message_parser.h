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

#ifndef NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H
#define NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H

#include <novatel_gps_driver/binary_message.h>
#include <novatel_gps_driver/nmea_sentence.h>
#include <novatel_gps_driver/novatel_sentence.h>

#include <novatel_gps_driver/parsers/parsing_utils.h>
#include <novatel_gps_driver/parsers/parse_exception.h>

#include <stdint.h>

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
   */
  template<typename T>
  class MessageParser
  {
  public:
    virtual ~MessageParser() {}

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
    virtual T ParseBinary(const BinaryMessage& bin_msg) throw(ParseException)
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
    virtual T ParseAscii(const NovatelSentence& sentence) throw(ParseException)
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
    virtual T ParseAscii(const NmeaSentence& sentence) throw(ParseException)
    {
      throw ParseException("ParseAscii not implemented.");
    };
  };
}

#endif //NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H
