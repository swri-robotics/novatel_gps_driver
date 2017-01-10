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

#ifndef NOVATEL_OEM628_NOVATEL_GPS_H_
#define NOVATEL_OEM628_NOVATEL_GPS_H_

#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <swri_serial_util/serial_port.h>

#include <gps_common/GPSFix.h>
#include <novatel_oem628/NovatelPosition.h>
#include <novatel_oem628/Gpgga.h>
#include <novatel_oem628/Gprmc.h>
#include <novatel_oem628/novatel_message_parser.h>

namespace novatel_oem628
{
  class NovatelGps
  {
    public:
      const static int32_t default_tcp_port_ = 3001;
      const static int32_t default_udp_port_ = 3002;

      enum ConnectionType { SERIAL, TCP, UDP, INVALID };

      enum ReadResult
      {
        READ_SUCCESS = 0,
        READ_INSUFFICIENT_DATA = 1,
        READ_TIMEOUT = 2,
        READ_INTERRUPTED = 3,
        READ_ERROR = -1,
        READ_PARSE_FAILED = -2
      };

      NovatelGps();
      ~NovatelGps();

      bool Connect(const std::string& device, ConnectionType connection);
      void Disconnect();

      static ConnectionType ParseConnection(const std::string& connection);

      ReadResult ProcessData();

      void GetNovatelPositions(std::vector<NovatelPositionPtr>& positions);
      void GetFixMessages(std::vector<gps_common::GPSFixPtr>& fix_messages);
      void GetGpggaMessages(std::vector<GpggaPtr>& gpgga_messages);
      void GetGprmcMessages(std::vector<GprmcPtr>& gprmc_messages);

      std::string ErrorMsg() const { return error_msg_; }

      //parameters
      double gpgga_gprmc_sync_tol; //seconds
      double gpgga_position_sync_tol; //seconds
      bool wait_for_position; //if false, do not require position message to make gps fix message
      //added this because position message is sometimes > 1 s late.
      void setBufferCapacity(const size_t buffer_size);

    private:
      bool CreateSerialConnection(const std::string& device);
      bool CreateTcpConnection(const std::string& device);
      bool CreateUdpConnection(const std::string& device);

      bool Write(const std::string& command);
      bool Configure();

      ReadResult ReadData();

      ConnectionType connection_;

      int32_t utc_offset_;

      // Serial
      swri_serial_util::SerialPort serial_;

      // TCP / UDP
      boost::asio::io_service io_service_;
      boost::asio::ip::tcp::socket tcp_socket_;
      boost::asio::ip::tcp::socket udp_socket_;

      // Data buffers
      std::vector<uint8_t> data_buffer_;
      std::string nmea_buffer_;
      std::vector<NmeaSentence> nmea_sentences_;
      std::vector<NovatelSentence> novatel_sentences_;

      // Message buffers
      boost::circular_buffer<NovatelPositionPtr> novatel_positions_;
      boost::circular_buffer<GpggaPtr> gpgga_msgs_;
      boost::circular_buffer<GprmcPtr> gprmc_msgs_;

      boost::circular_buffer<Gpgga> gpgga_sync_buffer_;
      boost::circular_buffer<Gprmc> gprmc_sync_buffer_;
      boost::circular_buffer<NovatelPositionPtr> position_sync_buffer_;

      std::string error_msg_;
  };
}

#endif  // NOVATEL_OEM628_NOVATEL_GPS_H_
