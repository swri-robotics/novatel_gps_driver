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

#ifndef NOVATEL_GPS_DRIVER_NOVATEL_GPS_H_
#define NOVATEL_GPS_DRIVER_NOVATEL_GPS_H_

// std libraries
#include <chrono>
#include <map>
#include <queue>
#include <string>
#include <vector>

// Boost
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// Messages
#include <gps_msgs/msg/gps_fix.hpp>

#include <novatel_gps_msgs/msg/gpgga.hpp>
#include <novatel_gps_msgs/msg/gprmc.hpp>

#include <sensor_msgs/msg/imu.hpp>

// Other libraries
#include <swri_serial_util/serial_port.h>

#include <pcap.h>

// Local include files
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
#include <novatel_gps_driver/parsers/gphdt.h>
#include <novatel_gps_driver/parsers/gprmc.h>
#include <novatel_gps_driver/parsers/heading2.h>
#include <novatel_gps_driver/parsers/dual_antenna_heading.h>
#include <novatel_gps_driver/parsers/inscov.h>
#include <novatel_gps_driver/parsers/inspva.h>
#include <novatel_gps_driver/parsers/inspvax.h>
#include <novatel_gps_driver/parsers/insstdev.h>
#include <novatel_gps_driver/parsers/range.h>
#include <novatel_gps_driver/parsers/psrdop2.h>
#include <novatel_gps_driver/parsers/time.h>
#include <novatel_gps_driver/parsers/trackstat.h>

namespace novatel_gps_driver
{
  /// Define NovatelMessageOpts as a map from message name to log period (seconds)
  /// A negative period will be logged as "onchanged" rather than "ontime"
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

      explicit NovatelGps(rclcpp::Node& Node);
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
      void GetFixMessages(std::vector<gps_msgs::msg::GPSFix::UniquePtr>& fix_messages);
      /**
       * @brief Provides any GPGGA messages that have been received since the
       * last time this was called.
       * @param[out] gpgga_messages New GPGGA messages.
       */
      void GetGpggaMessages(std::vector<novatel_gps_driver::GpggaParser::MessageType>& gpgga_messages);
      /**
       * @brief Provides any GPGSA messages that have been received since the
       * last time this was called.
       * @param[out] gpgsa_messages New GPGSA messages.
       */
      void GetGpgsaMessages(std::vector<novatel_gps_driver::GpgsaParser::MessageType>& gpgsa_messages);
      /**
       * @brief Provides any GPGSV messages that have been received since the
       * last time this was called.
       * @param[out] gpgsv_messages New GPGSV messages.
       */
      void GetGpgsvMessages(std::vector<novatel_gps_driver::GpgsvParser::MessageType>& gpgsv_messages);
      /**
       * @brief Provides any GPHDT messages that have been received since the
       * last time this was called.
       * @param[out] gpgsv_messages New GPHDT messages.
       */
      void GetGphdtMessages(std::vector<novatel_gps_driver::GphdtParser::MessageType>& gphdt_messages);
      /**
       * @brief Provides any GPRMC messages that have been received since the
       * last time this was called.
       * @param[out] gprmc_messages New GPRMC messages.
       */
      void GetGprmcMessages(std::vector<novatel_gps_driver::GprmcParser::MessageType>& gprmc_messages);
      /**
       * @brief Provides any HEADING2 messages that have been received since the
       * last time this was called.
       * @param[out] headings New HEADING2 messages.
       */
      void GetNovatelHeading2Messages(std::vector<novatel_gps_driver::Heading2Parser::MessageType>& headings);
      /**
       * @brief Provides any DUALANTENNAHEADING messages that have been received since the
       * last time this was called.
       * @param[out] headings New DUALANTENNAHEADING messages.
       */
      void GetNovatelDualAntennaHeadingMessages(std::vector<novatel_gps_driver::DualAntennaHeadingParser::MessageType>& headings);
      /**
       * @brief Provides any PSRDOP2 messages that have been received since the last time this
       * was called.
       * @param[out] psrdop2_messages New PSRDOP2 messages.
       */
       void GetNovatelPsrdop2Messages(std::vector<novatel_gps_driver::Psrdop2Parser::MessageType>& psrdop2_messages);
      /**
       * @brief Provides any Imu messages that have been generated since the
       * last time this was called.
       * @param[out] imu_message New Imu messages.
       */
      void GetImuMessages(std::vector<sensor_msgs::msg::Imu::SharedPtr>& imu_messages);
      /**
       * @brief Provides any INSCOV messages that have been received since the last
       * time this was called.
       * @param[out] inscov_messages New INSCOV messages.
       */
      void GetInscovMessages(std::vector<novatel_gps_driver::InscovParser::MessageType>& inscov_messages);
      /**
       * @brief Provides any INSPVA messages that have been received since the last
       * time this was called.
       * @param[out] inspva_messages New INSPVA messages.
       */
      void GetInspvaMessages(std::vector<novatel_gps_driver::InspvaParser::MessageType>& inspva_messages);
      /**
       * @brief Provides any INSPVAX messages that have been received since the last
       * time this was called.
       * @param[out] inspvax_messages New INSPVAX messages.
       */
      void GetInspvaxMessages(std::vector<novatel_gps_driver::InspvaxParser::MessageType>& inspvax_messages);
      /**
       * @brief Provides any INSSTDEV messages that have been received since the last
       * time this was called.
       * @param[out] insstdev_messages New INSSTDEV messages.
       */
      void GetInsstdevMessages(std::vector<novatel_gps_driver::InsstdevParser::MessageType>& insstdev_messages);
      /**
       * @brief Provides any CORRIMUDATA messages that have been received since the
       * last time this was called.
       * @param[out] imu_messages New CORRIMUDATA messages.
       */
      void GetNovatelCorrectedImuData(std::vector<novatel_gps_driver::CorrImuDataParser::MessageType>& imu_messages);
      /**
       * @brief Provides any BESTPOS messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTPOS messages.
       */
      void GetNovatelPositions(std::vector<novatel_gps_driver::BestposParser::MessageType>& positions);
      /**
       * @brief Provides any BESTXYZ messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTXYZ messages.
       */
      void GetNovatelXYZPositions(std::vector<novatel_gps_driver::BestxyzParser::MessageType>& positions);
      /**
       * @brief Provides any BESTUTM messages that have been received since the
       * last time this was called.
       * @param[out] positions New BESTUTM messages.
       */
      void GetNovatelUtmPositions(std::vector<novatel_gps_driver::BestutmParser::MessageType>& utm_positions);
      /**
       * @brief Provides any BESTVEL messages that have been received since the
       * last time this was called.
       * @param[out] velocities New BESTVEL messages.
       */
      void GetNovatelVelocities(std::vector<novatel_gps_driver::BestvelParser::MessageType>& velocities);
      /**
       * @brief Provides any RANGE messages that have been received since the
       * last time this was called.
       * @param[out] range_messages New RANGE messages.
       */
      void GetRangeMessages(std::vector<novatel_gps_driver::RangeParser::MessageType>& range_messages);
      /**
       * @brief Provides any TIME messages that have been received since the
       * last time this was called.
       * @param[out] time_messages New TIME messages.
       */
      void GetTimeMessages(std::vector<novatel_gps_driver::TimeParser::MessageType>& time_messages);
      /**
       * @brief Provides any TRACKSTAT messages that have been received since the
       * last time this was called.
       * @param[out] trackstat_msgs New TRACKSTAT messages.
       */
      void GetTrackstatMessages(std::vector<novatel_gps_driver::TrackstatParser::MessageType>& trackstat_msgs);
      /**
       * @brief Provides any CLOCKSTEERING messages that have been received since the
       * last time this was called.
       * @param[out] clocksteering_msgs New CLOCKSTEERING messages.
       */
      void GetClockSteeringMessages(std::vector<novatel_gps_driver::ClockSteeringParser::MessageType>& clocksteering_msgs);

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

      double gpsfix_sync_tol_; //seconds
      bool wait_for_sync_; // wait until a bestvel has arrived before publishing bestpos

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

      template<typename Input, typename Output>
      inline void DrainQueue(Input& in, Output& out)
      {
        out.clear();
        std::move(std::make_move_iterator(in.begin()),
            std::make_move_iterator(in.end()),
            std::back_inserter(out));
        in.clear();
      }

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
                                                const rclcpp::Time& stamp) noexcept(false);
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
                                               const rclcpp::Time& stamp,
                                               double most_recent_utc_time) noexcept(false);
      /**
       * @brief Converts a NovatelSentence object into a ROS message of the appropriate type
       * and places it in the appropriate buffer.
       * @param[in] msg A valid ASCII NovAtel message message
       * @param[in] stamp A timestamp to set in the ROS message header.
       * @return A value indicating the success of the operation.
       */
      NovatelGps::ReadResult ParseNovatelSentence(const NovatelSentence& sentence,
                                                  const rclcpp::Time& stamp) noexcept(false);

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


      rclcpp::Node& node_;

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
      std::shared_ptr<boost::asio::ip::udp::socket> udp_socket_;
      std::shared_ptr<boost::asio::ip::udp::endpoint> udp_endpoint_;

      // Data buffers
      /// Variable-length buffer that has data continually appended to it
      /// until messages are parsed from it
      std::vector<uint8_t> data_buffer_;
      /// Buffer containing incomplete data from message parsing
      std::string nmea_buffer_;
      /// Fixed-size buffer for reading directly from sockets
      std::array<uint8_t, 10000> socket_buffer_;

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
      DualAntennaHeadingParser dual_antenna_heading_parser_;
      ClockSteeringParser clocksteering_parser_;
      CorrImuDataParser corrimudata_parser_;
      GpggaParser gpgga_parser_;
      GpgsaParser gpgsa_parser_;
      GpgsvParser gpgsv_parser_;
      GphdtParser gphdt_parser_;
      GprmcParser gprmc_parser_;
      InscovParser inscov_parser_;
      InspvaParser inspva_parser_;
      InspvaxParser inspvax_parser_;
      InsstdevParser insstdev_parser_;
      Psrdop2Parser psrdop2_parser_;
      RangeParser range_parser_;
      TimeParser time_parser_;
      TrackstatParser trackstat_parser_;

      // Message buffers
      boost::circular_buffer<novatel_gps_driver::ClockSteeringParser::MessageType> clocksteering_msgs_;
      boost::circular_buffer<novatel_gps_driver::CorrImuDataParser::MessageType> corrimudata_msgs_;
      boost::circular_buffer<novatel_gps_driver::GpggaParser::MessageType> gpgga_msgs_;
      boost::circular_buffer<novatel_gps_driver::GpgsaParser::MessageType> gpgsa_msgs_;
      boost::circular_buffer<novatel_gps_driver::GpgsvParser::MessageType> gpgsv_msgs_;
      boost::circular_buffer<novatel_gps_driver::GphdtParser::MessageType> gphdt_msgs_;
      boost::circular_buffer<novatel_gps_driver::GprmcParser::MessageType> gprmc_msgs_;
      boost::circular_buffer<sensor_msgs::msg::Imu::SharedPtr> imu_msgs_;
      boost::circular_buffer<novatel_gps_driver::InscovParser::MessageType> inscov_msgs_;
      boost::circular_buffer<novatel_gps_driver::InspvaParser::MessageType> inspva_msgs_;
      boost::circular_buffer<novatel_gps_driver::InspvaxParser::MessageType> inspvax_msgs_;
      boost::circular_buffer<novatel_gps_driver::InsstdevParser::MessageType> insstdev_msgs_;
      boost::circular_buffer<novatel_gps_driver::BestposParser::MessageType> novatel_positions_;
      boost::circular_buffer<novatel_gps_driver::BestxyzParser::MessageType> novatel_xyz_positions_;
      boost::circular_buffer<novatel_gps_driver::BestutmParser::MessageType> novatel_utm_positions_;
      boost::circular_buffer<novatel_gps_driver::BestvelParser::MessageType> novatel_velocities_;
      boost::circular_buffer<novatel_gps_driver::BestposParser::MessageType> bestpos_sync_buffer_;
      boost::circular_buffer<novatel_gps_driver::BestvelParser::MessageType> bestvel_sync_buffer_;
      boost::circular_buffer<novatel_gps_driver::Heading2Parser::MessageType> heading2_msgs_;
      boost::circular_buffer<novatel_gps_driver::DualAntennaHeadingParser::MessageType> dual_antenna_heading_msgs_;
      boost::circular_buffer<novatel_gps_driver::Psrdop2Parser::MessageType> psrdop2_msgs_;
      boost::circular_buffer<novatel_gps_driver::RangeParser::MessageType> range_msgs_;
      boost::circular_buffer<novatel_gps_driver::TimeParser::MessageType> time_msgs_;
      boost::circular_buffer<novatel_gps_driver::TrackstatParser::MessageType> trackstat_msgs_;

      novatel_gps_driver::Psrdop2Parser::MessageType latest_psrdop2_;

      // IMU data synchronization queues
      std::queue<novatel_gps_driver::CorrImuDataParser::MessageType> corrimudata_queue_;
      std::queue<novatel_gps_driver::InspvaParser::MessageType> inspva_queue_;
      novatel_gps_driver::InsstdevParser::MessageType latest_insstdev_;
      novatel_gps_driver::InscovParser::MessageType latest_inscov_;
      double imu_rate_;

      // Additional Options
      bool apply_vehicle_body_rotation_;
  };
}

#endif  //NOVATEL_GPS_DRIVER_NOVATEL_GPS_H_
