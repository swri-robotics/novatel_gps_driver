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

#ifndef NOVATEL_GPS_DRIVER_BINARY_HEADER_H
#define NOVATEL_GPS_DRIVER_BINARY_HEADER_H

#include <cstdint>

#include <novatel_gps_driver/parsers/parsing_utils.h>

namespace novatel_gps_driver
{
  /**
   * Represents the header of a binary NovAtel message.
   */
  struct BinaryHeader
  {
    BinaryHeader() :
        sync0_(0xAA),
        sync1_(0x44),
        sync2_(0x12),
        header_length_(0),
        message_id_(0),
        message_type_(0),
        port_address_(0),
        message_length_(0),
        sequence_(0),
        idle_time_(0),
        time_status_(0),
        week_(0),
        gps_ms_(0),
        receiver_status_(0),
        reserved_(0),
        receiver_sw_version_(0)
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
    uint32_t gps_ms_;
    uint32_t receiver_status_;
    uint16_t reserved_;
    uint16_t receiver_sw_version_;

    void ParseHeader(const uint8_t* data)
    {
      sync0_ = data[0];
      sync1_ = data[1];
      sync2_ = data[2];
      header_length_ = data[3];
      message_id_ = ParseUInt16(&data[4]);
      message_type_ = data[6];
      port_address_ = data[7];
      message_length_ = ParseUInt16(&data[8]);
      sequence_ = ParseUInt16(&data[10]);
      idle_time_ = data[12];
      time_status_ = data[13];
      week_ = ParseUInt16(&data[14]);
      gps_ms_ = ParseUInt32(&data[16]);
      receiver_status_ = ParseUInt32(&data[20]);
      receiver_sw_version_ = ParseUInt16(&data[26]);
    }

    void ParseShortHeader(const uint8_t* data)
    {
      sync0_ = data[0];
      sync1_ = data[1];
      sync2_ = data[2];
      message_length_ = data[3];
      message_id_ = ParseUInt16(&data[4]);
      week_ = ParseUInt16(&data[6]);
      gps_ms_ = ParseUInt32(&data[8]);
      header_length_ = 12;  // TODO: HardCoding for now
    }
  };
}
#endif //NOVATEL_GPS_DRIVER_BINARY_HEADER_H
