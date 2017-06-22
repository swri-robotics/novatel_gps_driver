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

#include <map>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <swri_serial_util/serial_port.h>

#include <gps_common/GPSFix.h>
#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gpgsa.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>
#include <novatel_gps_msgs/Trackstat.h>
#include <novatel_oem628/novatel_message_parser.h>

namespace novatel_oem628
{
  /// Define NovatelMessageOpts as a map from message name to log period (seconds)
  typedef std::map<std::string, double> NovatelMessageOpts;

  class NovatelGps
  {
    public:
      const static int32_t default_tcp_port_ = 3001;
      const static int32_t default_udp_port_ = 3002;

      const static size_t MAX_BUFFER_SIZE = 100;
      const static size_t MAX_SYNC_BUFFER_SIZE = 10;

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

      /**
       * Connect and configure with default message options.
       * @param device A path to a device file handle, e.g. /dev/TTYUSB0
       * @param connection The type of connection. Only SERIAL is supported
       * @return True on success
       */
      bool Connect(const std::string& device, ConnectionType connection);

      /**
       * Connect and configure with the given message options
       * @param device A path to a device file handle, e.g. /dev/TTYUSB0
       * @param connection The type of connection. Only SERIAL is supported
       * @param opts Message options to use
       * @return
       */
      bool Connect(const std::string& device, ConnectionType connection, NovatelMessageOpts const& opts);

      /**
       * (Re)configure the driver with a set of message options
       *
       * @param opts A map from message name to log period (seconds)
       *
       * @return True on successful configuration
       */
      bool Configure(NovatelMessageOpts const& opts);

      void Disconnect();

      static ConnectionType ParseConnection(const std::string& connection);

      ReadResult ProcessData();

      void GetNovatelCorrectedImuData(std::vector<novatel_gps_msgs::NovatelCorrectedImuDataPtr>& imu_messages);
      void GetNovatelPositions(std::vector<novatel_gps_msgs::NovatelPositionPtr>& positions);
      void GetNovatelVelocities(std::vector<novatel_gps_msgs::NovatelVelocityPtr>& velocities);
      void GetFixMessages(std::vector<gps_common::GPSFixPtr>& fix_messages);
      void GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr>& gpgga_messages);
      void GetGpgsaMessages(std::vector<novatel_gps_msgs::GpgsaPtr>& gpgsa_messages);
      void GetGpgsvMessages(std::vector<novatel_gps_msgs::GpgsvPtr>& gpgsv_messages);
      void GetGprmcMessages(std::vector<novatel_gps_msgs::GprmcPtr>& gprmc_messages);
      void GetRangeMessages(std::vector<novatel_gps_msgs::RangePtr>& range_messages);
      void GetTimeMessages(std::vector<novatel_gps_msgs::TimePtr>& time_messages);
      void GetTrackstatMessages(std::vector<novatel_gps_msgs::TrackstatPtr>& trackstat_msgs);

      std::string ErrorMsg() const { return error_msg_; }

      //parameters
      double gpgga_gprmc_sync_tol; //seconds
      double gpgga_position_sync_tol; //seconds
      bool wait_for_position; //if false, do not require position message to make gps fix message
      //added this because position message is sometimes > 1 s late.
      void setBufferCapacity(const size_t buffer_size);

    private:
      bool CreateSerialConnection(const std::string& device, NovatelMessageOpts const& opts);
      bool CreateTcpConnection(const std::string& device, NovatelMessageOpts const& opts);
      bool CreateUdpConnection(const std::string& device, NovatelMessageOpts const& opts);

      NovatelGps::ReadResult ParseBinaryMessage(const BinaryMessage& msg,
                                                const ros::Time& stamp);
      NovatelGps::ReadResult ParseNmeaSentence(const NmeaSentence& sentence,
                                               const ros::Time& stamp,
                                               double most_recent_utc_time);
      NovatelGps::ReadResult ParseNovatelSentence(const NovatelSentence& sentence,
                                                  const ros::Time& stamp);

      bool Write(const std::string& command);

      ReadResult ReadData();

      ConnectionType connection_;

      double utc_offset_;

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
      std::vector<BinaryMessage> binary_messages_;

      // Message buffers
      boost::circular_buffer<novatel_gps_msgs::GpggaPtr> gpgga_msgs_;
      boost::circular_buffer<novatel_gps_msgs::Gpgga> gpgga_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::GpgsaPtr> gpgsa_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GpgsvPtr> gpgsv_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GprmcPtr> gprmc_msgs_;
      boost::circular_buffer<novatel_gps_msgs::Gprmc> gprmc_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::NovatelCorrectedImuDataPtr> imu_messages_;
      boost::circular_buffer<novatel_gps_msgs::NovatelPositionPtr> novatel_positions_;
      boost::circular_buffer<novatel_gps_msgs::NovatelVelocityPtr> novatel_velocities_;
      boost::circular_buffer<novatel_gps_msgs::NovatelPositionPtr> position_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::RangePtr> range_msgs_;
      boost::circular_buffer<novatel_gps_msgs::TimePtr> time_msgs_;
      boost::circular_buffer<novatel_gps_msgs::TrackstatPtr> trackstat_msgs_;

      std::string error_msg_;
  };
}

#endif  // NOVATEL_OEM628_NOVATEL_GPS_H_
