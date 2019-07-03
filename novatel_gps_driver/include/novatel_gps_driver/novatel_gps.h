// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#ifndef NOVATEL_OEM628_NOVATEL_GPS_H_
#define NOVATEL_OEM628_NOVATEL_GPS_H_

#include <map>
#include <queue>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <pcap.h>

#include <swri_serial_util/serial_port.h>

#include <gps_common/GPSFix.h>

#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gpgsa.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/Insstdev.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelXYZ.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>
#include <novatel_gps_msgs/Trackstat.h>

#include <novatel_gps_driver/novatel_message_extractor.h>

#include <novatel_gps_driver/parsers/bestpos.h>
#include <novatel_gps_driver/parsers/bestxyz.h>
#include <novatel_gps_driver/parsers/bestutm.h>
#include <novatel_gps_driver/parsers/bestvel.h>
#include <novatel_gps_driver/parsers/clocksteering.h>
#include <novatel_gps_driver/parsers/corrimudata.h>
#include <novatel_gps_driver/parsers/gpgga.h>
#include <novatel_gps_driver/parsers/gpgsa.h>
#include <novatel_gps_driver/parsers/gpgsv.h>
#include <novatel_gps_driver/parsers/gprmc.h>
#include <novatel_gps_driver/parsers/heading2.h>
#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/insstdev.h>
#include <novatel_gps_driver/parsers/range.h>
#include <novatel_gps_driver/parsers/time.h>
#include <novatel_gps_driver/parsers/trackstat.h>

#include <sensor_msgs/Imu.h>
#include <novatel_gps_driver/parsers/inscov.h>

namespace novatel_gps_driver
{
  /// Define NovatelMessageOpts as a map from message name to log period (seconds)
  typedef std::map<std::string, double> NovatelMessageOpts;

  class NovatelGps
  {
    public:
      enum ConnectionType { SERIAL, TCP, UDP, PCAP, INVALID };

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
       * @param device For serial connections, a path to a device file handle,
       * e.g. /dev/TTYUSB0; for IP connections, a host:port specification,
       * e.g. "192.168.1.10:3001"
       * @param connection The type of connection.
       * @return True on success
       */
      bool Connect(const std::string& device, ConnectionType connection);

      /**
       * Connect and configure with the given message options
       * @param device For serial connections, a path to a device file handle,
       * e.g. /dev/TTYUSB0; for IP connections, a host:port specification,
       * e.g. "192.168.1.10:3001"
       * @param connection The type of connection.
       * @param opts Message options to use
       * @return True on success
       */
      bool Connect(const std::string& device, ConnectionType connection, NovatelMessageOpts const& opts);

      /**
       * Disconnects from a connected device
       */
      void Disconnect();

      /**
       * @return The most recent error message
       */
      std::string ErrorMsg() const { return error_msg_; }

      /**
       * @brief Provides any GPSFix messages that have been generated since the
       * last time this was called.
       * @param[out] fix_messages New GPSFix messages.
       */
      void GetFixMessages(std::vector<gps_common::GPSFixPtr>& fix_messages);
      /**
       * @brief Provides any GPGGA messages that have been received since the
       * last time this was called.
       * @param[out] gpgga_messages New GPGGA messages.
       */
      void GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr>& gpgga_messages);
      /**
       * @brief Provides any GPGSA messages that have been received since the
       * last time this was called.
       * @param[out] gpgsa_messages New GPGSA messages.
       */
      void GetGpgsaMessages(std::vector<novatel_gps_msgs::GpgsaPtr>& gpgsa_messages);
      /**
       * @brief Provides any GPGSV messages that have been received since the
       * last time this was called.
       * @param[out] gpgsv_messages New GPGSV messages.
       */
      void GetGpgsvMessages(std::vector<novatel_gps_msgs::GpgsvPtr>& gpgsv_messages);
      /**
       * @brief Provides any GPRMC messages that have been received since the
       * last time this was called.
       * @param[out] gprmc_messages New GPRMC messages.
       */
      void GetGprmcMessages(std::vector<novatel_gps_msgs::GprmcPtr>& gprmc_messages);
      /**
       * @brief Provides any HEADING2 messages that have been received since the
       * last time this was called.
       * @param[out] headings New HEADING2 messages.
       */
      void GetNovatelHeading2Messages(std::vector<novatel_gps_msgs::NovatelHeading2Ptr>& headings);
      /**
       * @brief Provides any Imu messages that have been generated since the
       * last time this was called.
       * @param[out] imu_message New Imu messages.
       */
      void GetImuMessages(std::vector<sensor_msgs::ImuPtr>& imu_messages);
      /**
       * @brief Provides any INSCOV messages that have been received since the last
       * time this was called.
       * @param[out] inscov_messages New INSCOV messages.
       */
      void GetInscovMessages(std::vector<novatel_gps_msgs::InscovPtr>& inscov_messages);
      /**
       * @brief Provides any INSPVA messages that have been received since the last
       * time this was called.
       * @param[out] inspva_messages New INSPVA messages.
       */
      void GetInspvaMessages(std::vector<novatel_gps_msgs::InspvaPtr>& inspva_messages);
      /**
       * @brief Provides any INSSTDEV messages that have been received since the last
       * time this was called.
       * @param[out] insstdev_messages New INSSTDEV messages.
       */
      void GetInsstdevMessages(std::vector<novatel_gps_msgs::InsstdevPtr>& insstdev_messages);
      /**
       * @brief Provides any CORRIMUDATA messages that have been received since the
       * last time this was called.
       * @param[out] imu_messages New CORRIMUDATA messages.
       */
      void GetNovatelCorrectedImuData(std::vector<novatel_gps_msgs::NovatelCorrectedImuDataPtr>& imu_messages);
      /**
       * @brief Provides any BESTPOS messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTPOS messages.
       */
      void GetNovatelPositions(std::vector<novatel_gps_msgs::NovatelPositionPtr>& positions);
      /**
       * @brief Provides any BESTXYZ messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTXYZ messages.
       */
      void GetNovatelXYZPositions(std::vector<novatel_gps_msgs::NovatelXYZPtr>& positions);
      /**
       * @brief Provides any BESTUTM messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTUTM messages.
       */
      void GetNovatelUtmPositions(std::vector<novatel_gps_msgs::NovatelUtmPositionPtr>& utm_positions);
      /**
       * @brief Provides any BESTVEL messages that have been received since the
       * last time this was called.
       * @param[out] velocities New BESTVEL messages.
       */
      void GetNovatelVelocities(std::vector<novatel_gps_msgs::NovatelVelocityPtr>& velocities);
      /**
       * @brief Provides any RANGE messages that have been received since the
       * last time this was called.
       * @param[out] range_messages New RANGE messages.
       */
      void GetRangeMessages(std::vector<novatel_gps_msgs::RangePtr>& range_messages);
      /**
       * @brief Provides any TIME messages that have been received since the
       * last time this was called.
       * @param[out] time_messages New TIME messages.
       */
      void GetTimeMessages(std::vector<novatel_gps_msgs::TimePtr>& time_messages);
      /**
       * @brief Provides any TRACKSTAT messages that have been received since the
       * last time this was called.
       * @param[out] trackstat_msgs New TRACKSTAT messages.
       */
      void GetTrackstatMessages(std::vector<novatel_gps_msgs::TrackstatPtr>& trackstat_msgs);
      /**
       * @brief Provides any CLOCKSTEERING messages that have been received since the
       * last time this was called.
       * @param[out] clocksteering_msgs New CLOCKSTEERING messages.
       */
      void GetClockSteeringMessages(std::vector<novatel_gps_msgs::ClockSteeringPtr>& clocksteering_msgs);

      /**
       * @return true if we are connected to a NovAtel device, false otherwise.
       */
      bool IsConnected() { return is_connected_; }

      /**
       * @brief Converts the strings "udp", "tcp", or "serial" into the corresponding enum values.
       * @param connection A string indicating the connection type.
       * @return The corresponding enum value.
       */
      static ConnectionType ParseConnection(const std::string& connection);

      /**
       * @brief Determines whether or not to apply a 90 degree counter-clockwise rotation about Z
       * to the Novatel SPAN frame to match up with the ROS coordinate frame.
       * @param apply_rotation A bool indicating whether or not to apply the rotation.
       */
      void ApplyVehicleBodyRotation(const bool& apply_rotation);

      /**
       * @brief Processes any data that has been received from the device since the last time
       * this message was called.  May result in any number of messages being placed in the
       * individual message buffers.
       * @return A code indicating the success of reading from the device.
       */
      ReadResult ProcessData();

      /**
       * @brief Sets the IMU rate; necessary for producing sensor_msgs/Imu messages.
       * @param imu_rate The IMU rate in Hz.
       * @param force If this value should be used instead of an autodetected one
       */
      void SetImuRate(double imu_rate, bool force = true);

      /**
       * @brief Sets the serial baud rate; should be called before configuring a serial connection.
       * @param serial_baud_rate The serial baud rate.
       */
      void SetSerialBaud(int32_t serial_baud);

      /**
       * @brief Writes the given string of characters to a connected NovAtel device.
       * @param command A string to transmit.
       * @return true if we successfully wrote all of the data, false otherwise.
       */
      bool Write(const std::string& command);

      //parameters
      double gpgga_gprmc_sync_tol_; //seconds
      double gpgga_position_sync_tol_; //seconds
      bool wait_for_position_; //if false, do not require position message to make gps fix message
      //added this because position message is sometimes > 1 s late.

    private:
      /**
       * @brief (Re)configure the driver with a set of message options
       *
       * @param opts A map from message name to log period (seconds)
       *
       * @return True on successful configuration
       */
      bool Configure(NovatelMessageOpts const& opts);

      /**
       * @brief Establishes an IP connection with a NovAtel device.
       *
       * Based on the current value in connection_, this will create either a TCP or UDP socket;
       * then, based on the value in endpoint, it will either create a connection to a NovAtel
       * device on a specific port or wait for a connection from one.  In any case, this method
       * will block until a connection is either established or failed.
       *
       * After it has been called, Configure() will be called on the device and provided opts in
       * order to configure which logs it will produce.  After that, ReadData() and Write() may
       * be used to communicate with the device.
       *
       * @param endpoint A host and port specification; e. g., "192.168.1.10:1000".  If the host
       * host is omitted, it will listen for a connection the specified port.  If the port is
       * omitted, DEFAULT_TCP_PORT will be used for TCP connections and DEFAULT_UDP_PORT
       * for UDP connections.
       * @param opts A map of logs and periods the device should produce.
       * @return false if it failed to create a connection, true otherwise.
       */
      bool CreateIpConnection(const std::string& endpoint, NovatelMessageOpts const& opts);

      /**
       * @brief Establishes a serial port connection with a NovAtel device.
       *
       * It will create a connection set at the baud set by SetSerialBaudRate(), no parity, 
       * no flow control, and 8N1 bits, then it will call Configure() on that connection.  After
       * this method has succeeded, RedData() and Write() can be used to communicate with
       * the device.
       *
       * @param device The device node to connect to; e. g., "/dev/ttyUSB0"
       * @param opts A map of logs and periods the device should produce.
       * @return false if it failed to create a connection, true otherwise.
       */
      bool CreateSerialConnection(const std::string& device, NovatelMessageOpts const& opts);

      /**
       * @brief Creates a pcap device for playing back recorded data.
       *
       * @param device
       * @param opts
       * @return
       */
      bool CreatePcapConnection(const std::string& device, NovatelMessageOpts const& opts);

      /**
       * @brief Processes any messages in our corrimudata & inspva queues in order to
       * generate Imu messages from them.
       */
      void GenerateImuMessages();

      /**
       * @brief Converts a BinaryMessage object into a ROS message of the appropriate type
       * and places it in the appropriate buffer.
       * @param[in] msg A valid binary message
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseBinaryMessage(const BinaryMessage& msg,
                                                const ros::Time& stamp) throw(ParseException);
      /**
       * @brief Converts an NMEA sentence into a ROS message of the appropriate type and
       * places it in the appropriate buffer.
       * @param[in] sentence A valid NMEA sentence
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @param most_recent_utc_time The most recently received time in any GPGGA or GPRMC
       * message; used to adjust timestamps in ROS message headers.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseNmeaSentence(const NmeaSentence& sentence,
                                               const ros::Time& stamp,
                                               double most_recent_utc_time) throw(ParseException);
      /**
       * @brief Converts a NovatelSentence object into a ROS message of the appropriate type
       * and places it in the appropriate buffer.
       * @param[in] msg A valid ASCII NovAtel message message
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseNovatelSentence(const NovatelSentence& sentence,
                                                  const ros::Time& stamp) throw(ParseException);

      /**
       * @brief Reads data from a connected NovAtel device.  Any read data will be appended to
       * data_buffer_.
       * @return The status of the read operation.
       */
      ReadResult ReadData();

      static constexpr uint16_t DEFAULT_TCP_PORT = 3001;
      static constexpr uint16_t DEFAULT_UDP_PORT = 3002;
      static constexpr size_t MAX_BUFFER_SIZE = 100;
      static constexpr size_t SYNC_BUFFER_SIZE = 10;
      static constexpr uint32_t SECONDS_PER_WEEK = 604800;
      static constexpr double IMU_TOLERANCE_S = 0.0002;
      static constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;

      ConnectionType connection_;

      std::string error_msg_;

      bool is_connected_;
      bool imu_rate_forced_;

      double utc_offset_;

      // Serial connection
      int32_t serial_baud_;
      swri_serial_util::SerialPort serial_;

      // TCP / UDP connections
      boost::asio::io_service io_service_;
      boost::asio::ip::tcp::socket tcp_socket_;
      boost::shared_ptr<boost::asio::ip::udp::socket> udp_socket_;
      boost::shared_ptr<boost::asio::ip::udp::endpoint> udp_endpoint_;

      // Data buffers
      /// Variable-length buffer that has data continually appended to it
      /// until messages are parsed from it
      std::vector<uint8_t> data_buffer_;
      /// Buffer containing incomplete data from message parsing
      std::string nmea_buffer_;
      /// Fixed-size buffer for reading directly from sockets
      boost::array<uint8_t, 10000> socket_buffer_;

      /// pcap device for testing
      pcap_t* pcap_;
      bpf_program pcap_packet_filter_;
      char pcap_errbuf_[MAX_BUFFER_SIZE];
      std::vector<uint8_t> last_tcp_packet_;

      /// Used to extract messages from the incoming data stream
      NovatelMessageExtractor extractor_;

      // Message parsers
      BestposParser bestpos_parser_;
      BestxyzParser bestxyz_parser_;
      BestutmParser bestutm_parser_;
      BestvelParser bestvel_parser_;
      Heading2Parser heading2_parser_;
      ClockSteeringParser clocksteering_parser_;
      CorrImuDataParser corrimudata_parser_;
      GpggaParser gpgga_parser_;
      GpgsaParser gpgsa_parser_;
      GpgsvParser gpgsv_parser_;
      GprmcParser gprmc_parser_;
      InscovParser inscov_parser_;
      InspvaParser inspva_parser_;
      InsstdevParser insstdev_parser_;
      RangeParser range_parser_;
      TimeParser time_parser_;
      TrackstatParser trackstat_parser_;

      // Message buffers
      boost::circular_buffer<novatel_gps_msgs::ClockSteeringPtr> clocksteering_msgs_;
      boost::circular_buffer<novatel_gps_msgs::NovatelCorrectedImuDataPtr> corrimudata_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GpggaPtr> gpgga_msgs_;
      boost::circular_buffer<novatel_gps_msgs::Gpgga> gpgga_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::GpgsaPtr> gpgsa_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GpgsvPtr> gpgsv_msgs_;
      boost::circular_buffer<novatel_gps_msgs::GprmcPtr> gprmc_msgs_;
      boost::circular_buffer<novatel_gps_msgs::Gprmc> gprmc_sync_buffer_;
      boost::circular_buffer<sensor_msgs::ImuPtr> imu_msgs_;
      boost::circular_buffer<novatel_gps_msgs::InscovPtr> inscov_msgs_;
      boost::circular_buffer<novatel_gps_msgs::InspvaPtr> inspva_msgs_;
      boost::circular_buffer<novatel_gps_msgs::InsstdevPtr> insstdev_msgs_;
      boost::circular_buffer<novatel_gps_msgs::NovatelPositionPtr> novatel_positions_;
      boost::circular_buffer<novatel_gps_msgs::NovatelXYZPtr> novatel_xyz_positions_;
      boost::circular_buffer<novatel_gps_msgs::NovatelUtmPositionPtr> novatel_utm_positions_;
      boost::circular_buffer<novatel_gps_msgs::NovatelVelocityPtr> novatel_velocities_;
      boost::circular_buffer<novatel_gps_msgs::NovatelPositionPtr> position_sync_buffer_;
      boost::circular_buffer<novatel_gps_msgs::NovatelHeading2Ptr> heading2_msgs_;
      boost::circular_buffer<novatel_gps_msgs::RangePtr> range_msgs_;
      boost::circular_buffer<novatel_gps_msgs::TimePtr> time_msgs_;
      boost::circular_buffer<novatel_gps_msgs::TrackstatPtr> trackstat_msgs_;

      // IMU data synchronization queues
      std::queue<novatel_gps_msgs::NovatelCorrectedImuDataPtr> corrimudata_queue_;
      std::queue<novatel_gps_msgs::InspvaPtr> inspva_queue_;
      novatel_gps_msgs::InsstdevPtr latest_insstdev_;
      novatel_gps_msgs::InscovPtr latest_inscov_;
      double imu_rate_;

      // Additional Options
      bool apply_vehicle_body_rotation_;
  };
}

#endif  // NOVATEL_OEM628_NOVATEL_GPS_H_
