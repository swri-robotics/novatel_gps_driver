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

#include "novatel_gps_driver/parsers/rxstatus.h"
#include <iomanip>
#include <sstream>
#include <net/ethernet.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <novatel_gps_driver/novatel_gps.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>
#include <rclcpp/node.hpp>

// TODO: Remove once support is dropped.
#ifdef USE_TF2_GEOMETRY_MSGS_HPP
#  include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#  include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace novatel_gps_driver
{
  NovatelGps::NovatelGps(rclcpp::Node& node) :
      gpsfix_sync_tol_(0.01),
      wait_for_sync_(true),
      node_(node),
      connection_(SERIAL),
      is_connected_(false),
      imu_rate_forced_(false),
      utc_offset_(0),
      serial_baud_(115200),
      tcp_socket_(io_service_),
      pcap_(nullptr),
      extractor_(node_.get_logger()),
      clocksteering_msgs_(MAX_BUFFER_SIZE),
      corrimudata_msgs_(MAX_BUFFER_SIZE),
      corrimus_msgs_(MAX_BUFFER_SIZE),
      gpgga_msgs_(MAX_BUFFER_SIZE),
      gpgsa_msgs_(MAX_BUFFER_SIZE),
      gpgsv_msgs_(MAX_BUFFER_SIZE),
      gphdt_msgs_(MAX_BUFFER_SIZE),
      gprmc_msgs_(MAX_BUFFER_SIZE),
      imu_msgs_(MAX_BUFFER_SIZE),
      inscov_msgs_(MAX_BUFFER_SIZE),
      inspva_msgs_(MAX_BUFFER_SIZE),
      inspvas_msgs_(MAX_BUFFER_SIZE),
      inspvax_msgs_(MAX_BUFFER_SIZE),
      insstdev_msgs_(MAX_BUFFER_SIZE),
      novatel_positions_(MAX_BUFFER_SIZE),
      novatel_xyz_positions_(MAX_BUFFER_SIZE),
      novatel_utm_positions_(MAX_BUFFER_SIZE),
      novatel_velocities_(MAX_BUFFER_SIZE),
      bestpos_sync_buffer_(SYNC_BUFFER_SIZE),
      bestvel_sync_buffer_(SYNC_BUFFER_SIZE),
      heading2_msgs_(MAX_BUFFER_SIZE),
      dual_antenna_heading_msgs_(MAX_BUFFER_SIZE),
      psrdop2_msgs_(MAX_BUFFER_SIZE),
      range_msgs_(MAX_BUFFER_SIZE),
      time_msgs_(MAX_BUFFER_SIZE),
      trackstat_msgs_(MAX_BUFFER_SIZE),
      rxstatus_msgs_(MAX_BUFFER_SIZE),
      imu_rate_(-1.0),
      apply_vehicle_body_rotation_(false)
  {
  }

  NovatelGps::~NovatelGps()
  {
    Disconnect();
  }

  bool NovatelGps::Connect(
      const std::string& device,
      ConnectionType connection)
  {
    NovatelMessageOpts opts;
    opts["gpgga"] = 0.05;
    opts["gprmc"] = 0.05;
    opts["bestposa"] = 0.05;
    opts["timea"] = 1.0;
    opts["rangea"] = 1;
    return Connect(device, connection, opts);
  }

  bool NovatelGps::Connect(
      const std::string& device,
      ConnectionType connection,
      NovatelMessageOpts const& opts)
  {
    Disconnect();

    connection_ = connection;

    if (connection_ == SERIAL)
    {
      return CreateSerialConnection(device, opts);
    }
    else if (connection_ == TCP || connection_ == UDP)
    {
      return CreateIpConnection(device, opts);
    }
    else if (connection_ == PCAP)
    {
      return CreatePcapConnection(device, opts);
    }

    error_msg_ = "Invalid connection type.";

    return false;

  }

  NovatelGps::ConnectionType NovatelGps::ParseConnection(const std::string& connection)
  {
    if (connection == "serial")
    {
      return SERIAL;
    }
    else if (connection == "udp")
    {
      return UDP;
    }
    else if (connection == "tcp")
    {
      return TCP;
    }
    else if (connection == "pcap")
    {
      return PCAP;
    }

    return INVALID;
  }

  void NovatelGps::Disconnect()
  {
    if (connection_ == SERIAL)
    {
      serial_.Close();
    }
    else if (connection_ == TCP)
    {
      tcp_socket_.close();
    }
    else if (connection_ == UDP)
    {
      if (udp_socket_)
      {
        udp_socket_->close();
        udp_socket_.reset();
      }
      if (udp_endpoint_)
      {
        udp_endpoint_.reset();
      }
    }
    else if (connection_ == PCAP)
    {
      if (pcap_ != nullptr)
      {
        pcap_close(pcap_);
        pcap_ = nullptr;
      }
    }
    is_connected_ = false;
  }

  void NovatelGps::ApplyVehicleBodyRotation(const bool& apply_rotation)
  {
    apply_vehicle_body_rotation_ = apply_rotation;
  }

  NovatelGps::ReadResult NovatelGps::ProcessData()
  {
    NovatelGps::ReadResult read_result = ReadData();

    if (read_result != READ_SUCCESS)
    {
      return read_result;
    }

    rclcpp::Time stamp = node_.get_clock()->now();
    std::vector<NmeaSentence> nmea_sentences;
    std::vector<NovatelSentence> novatel_sentences;
    std::vector<BinaryMessage> binary_messages;

    if (!data_buffer_.empty())
    {
      nmea_buffer_.insert(nmea_buffer_.end(),
                          data_buffer_.begin(),
                          data_buffer_.end());

      data_buffer_.clear();

      std::string remaining_buffer;

      if (!extractor_.ExtractCompleteMessages(
          nmea_buffer_,
          nmea_sentences,
          novatel_sentences,
          binary_messages,
          remaining_buffer))
      {
        read_result = READ_PARSE_FAILED;
        error_msg_ = "Parse failure extracting sentences.";
      }

      nmea_buffer_ = remaining_buffer;

      RCLCPP_DEBUG(node_.get_logger(), "Parsed: %lu NMEA / %lu NovAtel / %lu Binary messages",
               nmea_sentences.size(), novatel_sentences.size(), binary_messages.size());
      if (!nmea_buffer_.empty())
      {
        RCLCPP_DEBUG(node_.get_logger(), "%lu unparsed bytes left over.", nmea_buffer_.size());
      }
    }

    double most_recent_utc_time = extractor_.GetMostRecentUtcTime(nmea_sentences);

    for(const auto& sentence : nmea_sentences)
    {
      try
      {
        NovatelGps::ReadResult result = ParseNmeaSentence(sentence, stamp, most_recent_utc_time);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const ParseException& p)
      {
        error_msg_ = p.what();
        RCLCPP_WARN(node_.get_logger(), "%s", p.what());
        RCLCPP_WARN(node_.get_logger(), "For sentence: [%s]", boost::algorithm::join(sentence.body, ",").c_str());
        read_result = READ_PARSE_FAILED;
      }
    }

    for(const auto& sentence : novatel_sentences)
    {
      try
      {
        NovatelGps::ReadResult result = ParseNovatelSentence(sentence, stamp);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const ParseException& p)
      {
        error_msg_ = p.what();
        RCLCPP_WARN(node_.get_logger(), "%s", p.what());
        read_result = READ_PARSE_FAILED;
      }
    }

    for(const auto& msg : binary_messages)
    {
      try
      {
        NovatelGps::ReadResult result = ParseBinaryMessage(msg, stamp);
        if (result != READ_SUCCESS)
        {
          read_result = result;
        }
      }
      catch (const ParseException& p)
      {
        error_msg_ = p.what();
        RCLCPP_WARN(node_.get_logger(), "%s", p.what());
        read_result = READ_PARSE_FAILED;
      }
    }

    return read_result;
  }

  void NovatelGps::GetNovatelPositions(std::vector<novatel_gps_driver::BestposParser::MessageType>& positions)
  {
    positions.clear();
    positions.insert(positions.end(), novatel_positions_.begin(), novatel_positions_.end());
    novatel_positions_.clear();
  }

  void NovatelGps::GetNovatelXYZPositions(std::vector<novatel_gps_driver::BestxyzParser::MessageType>& positions)
  {
    DrainQueue(novatel_xyz_positions_, positions);
  }

  void NovatelGps::GetNovatelUtmPositions(std::vector<novatel_gps_driver::BestutmParser::MessageType>& utm_positions)
  {
    DrainQueue(novatel_utm_positions_, utm_positions);
  }

  void NovatelGps::GetNovatelVelocities(std::vector<novatel_gps_driver::BestvelParser::MessageType>& velocities)
  {
    DrainQueue(novatel_velocities_, velocities);
  }

  void NovatelGps::GetFixMessages(std::vector<gps_msgs::msg::GPSFix::UniquePtr>& fix_messages)
  {
    // Clear out the fix_messages list before filling it
    fix_messages.clear();

    while (!bestpos_sync_buffer_.empty())
    {
      auto& bestpos = bestpos_sync_buffer_.front();

      bool synced = false;

      auto gpsFix = std::make_unique<gps_msgs::msg::GPSFix>();

      // Speed & Track are filled in from BESTVEL logs, if available
      while (!bestvel_sync_buffer_.empty())
      {
        auto& bestvel = bestvel_sync_buffer_.front();

        // TODO pjr handle wrapping around week boundary?
        double time_diff = std::fabs(bestvel->novatel_msg_header.gps_seconds -
                                     bestpos->novatel_msg_header.gps_seconds);
        if (time_diff < gpsfix_sync_tol_)
        {
          // Within tolerance, messages synced
          gpsFix->track = bestvel->track_ground;
          gpsFix->speed = std::sqrt(std::pow(bestvel->horizontal_speed, 2) + std::pow(bestvel->vertical_speed, 2));
          synced = true;
          break;
        }
        else if (bestvel->novatel_msg_header.gps_seconds < bestpos->novatel_msg_header.gps_seconds)
        {
          // Bestvel timestamp is too old, throw it away and try again
          bestvel_sync_buffer_.pop_front();
        }
        else
        {
          // Latest bestvel message is too new, do nothing for now
          break;
        }
      }

      if (!synced && wait_for_sync_)
      {
        // TODO Handle this properly if bestvel is disabled
        // If we have a bestpos and are configured to wait for a sync, we need
        // to wait until a bestvel arrives.

        break;
      }

      gpsFix->header.stamp = node_.get_clock()->now();
      gpsFix->altitude = bestpos->height;
      gpsFix->latitude = bestpos->lat;
      gpsFix->longitude = bestpos->lon;

      if (bestpos->solution_status == "SOL_COMPUTED")
      {
        gpsFix->status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;
      }
      else
      {
        gpsFix->status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
      }

      gpsFix->time = bestpos->novatel_msg_header.gps_seconds;

      gpsFix->status.header.stamp = gpsFix->header.stamp;
      gpsFix->status.satellites_visible = bestpos->num_satellites_tracked;
      gpsFix->status.satellites_used = bestpos->num_satellites_used_in_solution;

      double sigma_x = bestpos->lon_sigma;
      double sigma_x_squared = sigma_x * sigma_x;
      gpsFix->position_covariance[0] = sigma_x_squared;

      double sigma_y = bestpos->lat_sigma;
      double sigma_y_squared = sigma_y * sigma_y;
      gpsFix->position_covariance[4] = sigma_y_squared;

      double sigma_z = bestpos->height_sigma;
      gpsFix->position_covariance[8] = sigma_z * sigma_z;

      // See https://www.novatel.com/assets/Documents/Bulletins/apn029.pdf for info about errors
      // 2D error is 2*DRMS, or 95% probability
      gpsFix->err_horz = 2.0 * std::sqrt(sigma_x_squared + sigma_y_squared);

      // 3D error is 90% spherical accuracy standard
      gpsFix->err = 0.833 * (sigma_x + sigma_y + sigma_z);

      // Vertical error is 2*sigma for 95% probability
      gpsFix->err_vert = 2.0 * sigma_z;

      gpsFix->position_covariance_type =
          gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      // DOPs are filled in from the PSRDOP2 log, if available
      if (latest_psrdop2_)
      {
        gpsFix->gdop = latest_psrdop2_->gdop;
        gpsFix->pdop = latest_psrdop2_->pdop;
        gpsFix->hdop = latest_psrdop2_->hdop;
        gpsFix->vdop = latest_psrdop2_->vdop;
        if (!latest_psrdop2_->systems.empty())
        {
          // The PSRDOP2 log can have multiple TDOPs, but the GPSFix message
          // only has one... so just use the first one.
          gpsFix->tdop = latest_psrdop2_->systems.front().tdop;
        }
      }

      // TODO Fill in orientation from INSATT on SPAN systems

      fix_messages.push_back(std::move(gpsFix));
      bestpos_sync_buffer_.pop_front();
    }
  }

  void  NovatelGps::GetNovatelHeading2Messages(std::vector<novatel_gps_driver::Heading2Parser::MessageType>& headings)
  {
    DrainQueue(heading2_msgs_, headings);
  }

  void  NovatelGps::GetNovatelDualAntennaHeadingMessages(
      std::vector<novatel_gps_driver::DualAntennaHeadingParser::MessageType>& headings)
  {
    DrainQueue(dual_antenna_heading_msgs_, headings);
  }

  void NovatelGps::GetNovatelCorrectedImuData(std::vector<novatel_gps_driver::CorrImuDataParser::MessageType>& imu_messages)
  {
    DrainQueue(corrimudata_msgs_, imu_messages);
  }

  void NovatelGps::GetNovatelCorrectedImus(std::vector<novatel_gps_driver::CorrImusParser::MessageType>& imu_messages)
  {
    DrainQueue(corrimus_msgs_, imu_messages);
  }

  void NovatelGps::GetNovatelPsrdop2Messages(std::vector<novatel_gps_driver::Psrdop2Parser::MessageType>& psrdop2_messages)
  {
    psrdop2_messages.clear();
    psrdop2_messages.insert(psrdop2_messages.end(), psrdop2_msgs_.begin(), psrdop2_msgs_.end());
    psrdop2_msgs_.clear();
  }

  void NovatelGps::GetGpggaMessages(std::vector<novatel_gps_driver::GpggaParser::MessageType>& gpgga_messages)
  {
    DrainQueue(gpgga_msgs_, gpgga_messages);
  }

  void NovatelGps::GetGpgsaMessages(std::vector<novatel_gps_driver::GpgsaParser::MessageType>& gpgsa_messages)
  {
    DrainQueue(gpgsa_msgs_, gpgsa_messages);
  }

  void NovatelGps::GetGpgsvMessages(std::vector<novatel_gps_driver::GpgsvParser::MessageType>& gpgsv_messages)
  {
    DrainQueue(gpgsv_msgs_, gpgsv_messages);
  }

  void NovatelGps::GetGphdtMessages(std::vector<novatel_gps_driver::GphdtParser::MessageType>& gphdt_messages)
  {
    DrainQueue(gphdt_msgs_, gphdt_messages);
  }

  void NovatelGps::GetGprmcMessages(std::vector<novatel_gps_driver::GprmcParser::MessageType>& gprmc_messages)
  {
    DrainQueue(gprmc_msgs_, gprmc_messages);
  }

  void NovatelGps::GetInscovMessages(std::vector<novatel_gps_driver::InscovParser::MessageType>& inscov_messages)
  {
    inscov_messages.clear();
    inscov_messages.insert(inscov_messages.end(), inscov_msgs_.begin(), inscov_msgs_.end());
    inscov_msgs_.clear();
  }

  void NovatelGps::GetInspvaMessages(std::vector<novatel_gps_driver::InspvaParser::MessageType>& inspva_messages)
  {
    DrainQueue(inspva_msgs_, inspva_messages);
  }

  void NovatelGps::GetInspvasMessages(std::vector<novatel_gps_driver::InspvasParser::MessageType>& inspvas_messages)
  {
    DrainQueue(inspvas_msgs_, inspvas_messages);
  }

  void NovatelGps::GetInspvaxMessages(std::vector<novatel_gps_driver::InspvaxParser::MessageType>& inspvax_messages)
  {
    DrainQueue(inspvax_msgs_, inspvax_messages);
  }

  void NovatelGps::GetInsstdevMessages(std::vector<novatel_gps_driver::InsstdevParser::MessageType>& insstdev_messages)
  {
    insstdev_messages.clear();
    insstdev_messages.insert(insstdev_messages.end(), insstdev_msgs_.begin(), insstdev_msgs_.end());
    insstdev_msgs_.clear();
  }

  void NovatelGps::GetRangeMessages(std::vector<novatel_gps_driver::RangeParser::MessageType>& range_messages)
  {
    DrainQueue(range_msgs_, range_messages);
  }

  void NovatelGps::GetTimeMessages(std::vector<novatel_gps_driver::TimeParser::MessageType>& time_messages)
  {
    DrainQueue(time_msgs_, time_messages);
  }

  void NovatelGps::GetTrackstatMessages(std::vector<novatel_gps_driver::TrackstatParser::MessageType>& trackstat_msgs)
  {
    DrainQueue(trackstat_msgs_, trackstat_msgs);
  }

  void NovatelGps::GetClockSteeringMessages(std::vector<novatel_gps_driver::ClockSteeringParser::MessageType>& clocksteering_msgs)
  {
    DrainQueue(clocksteering_msgs_, clocksteering_msgs);
  }

  void NovatelGps::GetRxStatusMessages(std::vector<novatel_gps_driver::RxStatusParser::MessageType>& rxstatus_msgs)
  {
    DrainQueue(rxstatus_msgs_, rxstatus_msgs);
  }

  bool NovatelGps::CreatePcapConnection(const std::string& device, NovatelMessageOpts const& opts)
  {
    RCLCPP_INFO(node_.get_logger(), "Opening pcap file: %s", device.c_str());

    if ((pcap_ = pcap_open_offline(device.c_str(), pcap_errbuf_)) == nullptr)
    {
      RCLCPP_FATAL(node_.get_logger(), "Unable to open pcap file.");
      return false;
    }

    pcap_compile(pcap_, &pcap_packet_filter_, "tcp dst port 3001", 1, PCAP_NETMASK_UNKNOWN);
    is_connected_ = true;

    return true;
  }

  bool NovatelGps::CreateSerialConnection(const std::string& device, NovatelMessageOpts const& opts)
  {
    swri_serial_util::SerialConfig config;
    config.baud = serial_baud_;
    config.parity = swri_serial_util::SerialConfig::NO_PARITY;
    config.flow_control = false;
    config.data_bits = 8;
    config.stop_bits = 1;
    config.low_latency_mode = false;
    config.writable = true; // Assume that we can write to this port

    bool success = serial_.Open(device, config);

    if (success)
    {
      is_connected_ = true;
      if (!Configure(opts))
      {
        // We will not kill the connection here, because the device may already
        // be setup to communicate correctly, but we will print a warning
        RCLCPP_ERROR(node_.get_logger(), "Failed to configure GPS. This port may be read only, or the "
                 "device may not be functioning as expected; however, the "
                 "driver may still function correctly if the port has already "
                 "been pre-configured.");
      }
    }
    else
    {
      error_msg_ = serial_.ErrorMsg();
    }

    return success;
  }

  bool NovatelGps::CreateIpConnection(const std::string& endpoint, NovatelMessageOpts const& opts)
  {
    std::string ip;
    std::string port;
    uint16_t num_port;
    size_t sep_pos = endpoint.find(':');
    if (sep_pos == std::string::npos || sep_pos == endpoint.size() - 1)
    {
      RCLCPP_INFO(node_.get_logger(), "Using default port.");
      std::stringstream ss;
      if (connection_ == TCP)
      {
        num_port = DEFAULT_TCP_PORT;
      }
      else
      {
        num_port = DEFAULT_UDP_PORT;
      }
      ss << num_port;
      port = ss.str();
    }
    else
    {
      port = endpoint.substr(sep_pos + 1);
    }

    if (sep_pos != 0)
    {
      ip = endpoint.substr(0, sep_pos);
    }

    try
    {
      if (!ip.empty())
      {
        if (connection_ == TCP)
        {
          boost::asio::ip::tcp::resolver resolver(io_service_);
          boost::asio::ip::tcp::resolver::query query(ip, port);
          boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

          boost::asio::connect(tcp_socket_, iter);
          RCLCPP_INFO(node_.get_logger(), "Connecting via TCP to %s:%s", ip.c_str(), port.c_str());
        }
        else
        {
          boost::asio::ip::udp::resolver resolver(io_service_);
          boost::asio::ip::udp::resolver::query query(ip, port);
          udp_endpoint_ = std::make_shared<boost::asio::ip::udp::endpoint>(*resolver.resolve(query));
          udp_socket_.reset(new boost::asio::ip::udp::socket(io_service_));
          udp_socket_->open(boost::asio::ip::udp::v4());
          RCLCPP_INFO(node_.get_logger(), "Connecting via UDP to %s:%s", ip.c_str(), port.c_str());
        }
      }
      else
      {
        auto port_num = static_cast<uint16_t>(strtoll(port.c_str(), nullptr, 10));
        if (connection_ == TCP)
        {
          boost::asio::ip::tcp::acceptor acceptor(io_service_,
                                                  boost::asio::ip::tcp::endpoint(
                                                      boost::asio::ip::tcp::v4(), port_num));
          RCLCPP_INFO(node_.get_logger(), "Listening on TCP port %s", port.c_str());
          acceptor.accept(tcp_socket_);
          RCLCPP_INFO(node_.get_logger(), "Accepted TCP connection from client: %s",
                   tcp_socket_.remote_endpoint().address().to_string().c_str());
        }
        else
        {
          udp_socket_.reset(new boost::asio::ip::udp::socket(
              io_service_,
              boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                             port_num)));
          std::array<char, 1> recv_buf;
          udp_endpoint_ = std::make_shared<boost::asio::ip::udp::endpoint>();
          boost::system::error_code error;

          RCLCPP_INFO(node_.get_logger(), "Listening on UDP port %s", port.c_str());
          udp_socket_->receive_from(boost::asio::buffer(recv_buf), *udp_endpoint_, 0, error);
          if (error && error != boost::asio::error::message_size)
          {
            throw boost::system::system_error(error);
          }

          RCLCPP_INFO(node_.get_logger(), "Accepted UDP connection from client: %s",
                   udp_endpoint_->address().to_string().c_str());
        }
      }
    }
    catch (std::exception& e)
    {
      error_msg_ = e.what();
      RCLCPP_ERROR(node_.get_logger(), "Unable to connect: %s", e.what());
      return false;
    }

    is_connected_ = true;

    if (Configure(opts))
    {
      RCLCPP_INFO(node_.get_logger(), "Configured GPS.");
    }
    else
    {
      // We will not kill the connection here, because the device may already
      // be setup to communicate correctly, but we will print a warning
      RCLCPP_ERROR(node_.get_logger(), "Failed to configure GPS. This port may be read only, or the "
                   "device may not be functioning as expected; however, the "
                   "driver may still function correctly if the port has already "
                   "been pre-configured.");
    }

    return true;
  }

  NovatelGps::ReadResult NovatelGps::ReadData()
  {
    if (connection_ == SERIAL)
    {
      swri_serial_util::SerialPort::Result result =
          serial_.ReadBytes(data_buffer_, 0, 1000);

      if (result == swri_serial_util::SerialPort::ERROR)
      {
        error_msg_ = serial_.ErrorMsg();
        return READ_ERROR;
      }
      else if (result == swri_serial_util::SerialPort::TIMEOUT)
      {
        error_msg_ = "Timed out waiting for serial device.";
        return READ_TIMEOUT;
      }
      else if (result == swri_serial_util::SerialPort::INTERRUPTED)
      {
        error_msg_ = "Interrupted during read from serial device.";
        return READ_INTERRUPTED;
      }

      return READ_SUCCESS;
    }
    else if (connection_ == TCP || connection_ == UDP)
    {
      try
      {
        boost::system::error_code error;
        size_t len;

        if (connection_ == TCP)
        {
          len = tcp_socket_.read_some(boost::asio::buffer(socket_buffer_), error);
        }
        else
        {
          boost::asio::ip::udp::endpoint remote_endpoint;
          len = udp_socket_->receive_from(boost::asio::buffer(socket_buffer_), remote_endpoint);
        }
        data_buffer_.insert(data_buffer_.end(), socket_buffer_.begin(), socket_buffer_.begin()+len);
        if (error)
        {
          error_msg_ = error.message();
          Disconnect();
          return READ_ERROR;
        }
        return READ_SUCCESS;
      }
      catch (std::exception& e)
      {
        RCLCPP_WARN(node_.get_logger(), "TCP connection error: %s", e.what());
      }
    }
    else if (connection_ == PCAP)
    {
      struct pcap_pkthdr* header;
      const u_char *pkt_data;

      int result;
      result = pcap_next_ex(pcap_, &header, &pkt_data);
      if (result >= 0)
      {
        auto iph = reinterpret_cast<const iphdr*>(pkt_data + sizeof(struct ethhdr));
        uint32_t iphdrlen = iph->ihl * 4u;

        switch (iph->protocol)
        {
          case 6: // TCP
          {
            if (header->len == 54)
            {
              // Empty packet, skip it.
              return READ_SUCCESS;
            }

            // It's possible to get multiple subsequent TCP packets with the same seq
            // but latter ones have more data than previous ones.  In case that happens,
            // we need to only process the one with the most data.  We do that by
            // storing the most recently received message in a buffer, replacing it if
            // we get a new one with the same seq but more data, and only sending it to
            // the parser when we get a new packet with a different seq.
            // Note that when we copy data into last_tcp_packet_, we omit the ethernet
            // header because we don't care about it; we still need the IP and TCP
            // headers.  After we move it from last_tcp_packet_ into data_buffer, we
            // can skip the IP header and the TCP data offset.
            bool store_packet = true;
            if (!last_tcp_packet_.empty())
            {
              auto tcph = reinterpret_cast<const tcphdr*>(pkt_data + iphdrlen + sizeof(struct ethhdr));
              auto last_iph = reinterpret_cast<const iphdr*>(&(last_tcp_packet_[0]));
              uint32_t last_iphdrlen = last_iph->ihl * 4u;
              auto last_tcph = reinterpret_cast<const tcphdr*>(&(last_tcp_packet_[0]) + last_iphdrlen);
              uint16_t last_len = ntohs(static_cast<uint16_t>(last_iph->tot_len));
              uint16_t new_len = ntohs(static_cast<uint16_t>(iph->tot_len));
              uint32_t last_seq = ntohl(last_tcph->seq);
              uint32_t new_seq = ntohl(tcph->seq);

              if (new_seq != last_seq)
              {
                // If we got a packet that has a different seq than our previous one, send
                // the previous one and store the new one.
                uint32_t data_offset = last_tcph->doff * 4;
                data_buffer_.insert(data_buffer_.end(),
                                    last_tcp_packet_.begin() + last_iphdrlen + data_offset,
                                    last_tcp_packet_.end());
              }
              else if (new_len <= last_len)
              {
                // If we got a packet with the same seq as the previous one but it doesn't
                // have more data, do nothing.
                store_packet = false;
              }
            }

            if (store_packet)
            {
              // If we get here, we either just sent the previous packet, or we got
              // a new packet with the same seq but more data.  In either case,
              // store it.
              last_tcp_packet_.clear();
              last_tcp_packet_.insert(last_tcp_packet_.end(),
                                      pkt_data + sizeof(struct ethhdr),
                                      pkt_data + header->len);
            }

            break;
          }
          case 17: // UDP
          {
            uint16_t frag_off = ntohs(static_cast<uint16_t>(iph->frag_off));
            uint16_t fragment_offset = frag_off & static_cast<uint16_t>(0x1FFF);
            size_t header_size;
            // UDP packets may be fragmented; this isn't really "correct", but for
            // simplicity's sake we'll assume we get fragments in the right order.
            if (fragment_offset == 0)
            {
              header_size = sizeof(struct ethhdr) + iphdrlen + sizeof(struct udphdr);
            }
            else
            {
              header_size = sizeof(struct ethhdr) + iphdrlen;
            }

            data_buffer_.insert(data_buffer_.end(), pkt_data + header_size, pkt_data + header->len);

            break;
          }
          default:
            RCLCPP_WARN(node_.get_logger(), "Unexpected protocol: %u", iph->protocol);
            return READ_ERROR;
        }

        // Add a slight delay after reading packets; if the node is being tested offline
        // and this loop is hammering the TCP, logs won't output properly.
        rclcpp::sleep_for(std::chrono::milliseconds(1));

        return READ_SUCCESS;
      }
      else if (result == -2)
      {
        RCLCPP_INFO(node_.get_logger(), "Done reading pcap file.");
        if (!last_tcp_packet_.empty())
        {
          // Don't forget to submit the last packet if we still have one!
          auto last_iph = reinterpret_cast<const iphdr*>(&(last_tcp_packet_[0]));
          uint32_t iphdrlen = last_iph->ihl * 4u;
          auto last_tcph = reinterpret_cast<const tcphdr*>(&(last_tcp_packet_[0]) + iphdrlen);
          uint32_t data_offset = last_tcph->doff * 4u;
          data_buffer_.insert(data_buffer_.end(),
                              last_tcp_packet_.begin() + iphdrlen + data_offset,
                              last_tcp_packet_.end());
          last_tcp_packet_.clear();
        }
        Disconnect();
        return READ_SUCCESS;
      }
      else
      {
        RCLCPP_WARN(node_.get_logger(), "Error reading pcap data: %s", pcap_geterr(pcap_));
        return READ_ERROR;
      }
    }

    error_msg_ = "Unsupported connection type.";

    return READ_ERROR;
  }

  void NovatelGps::GetImuMessages(std::vector<sensor_msgs::msg::Imu::SharedPtr>& imu_messages)
  {
    imu_messages.clear();
    imu_messages.insert(imu_messages.end(), imu_msgs_.begin(), imu_msgs_.end());
    imu_msgs_.clear();
  }

  void NovatelGps::GenerateImuMessages()
  {
    if (imu_rate_ <= 0.0)
    {
      RCLCPP_WARN_ONCE(node_.get_logger(), "IMU rate has not been configured; cannot produce sensor_msgs/Imu messages.");
      return;
    }

    if (!latest_insstdev_ && !latest_inscov_)
    {
      // TODO pjr Make this a _THROTTLE log when it's available
      // If we haven't received an INSSTDEV or an INSCOV message, don't do anything, just return.
      RCLCPP_WARN(node_.get_logger(), "No INSSTDEV or INSCOV data yet; orientation covariance will be unavailable.");
    }

    size_t previous_size = imu_msgs_.size();
    // Only do anything if we have both CORRIMUDATA and INSPVA messages.
    while ((!corrimudata_queue_.empty() || !corrimus_queue_.empty()) && (!inspva_queue_.empty() || !inspvas_queue_.empty()))
    {
      
      const auto& corrimudata = !corrimudata_queue_.empty() ? corrimudata_queue_.front() : corrimus_queue_.front();
      const auto& inspva = !inspva_queue_.empty() ? inspva_queue_.front() : inspvas_queue_.front();
      
      double corrimudata_time = corrimudata->gps_week_num * SECONDS_PER_WEEK + corrimudata->gps_seconds;
      double inspva_time = inspva->novatel_msg_header.gps_week_num *
                               SECONDS_PER_WEEK + inspva->novatel_msg_header.gps_seconds;

      if (std::fabs(corrimudata_time - inspva_time) > IMU_TOLERANCE_S)
      {
        // If the two messages are too far apart to sync, discard the oldest one.
        RCLCPP_DEBUG(node_.get_logger(), "INSPVA and CORRIMUDATA were unacceptably far apart.");
        if (corrimudata_time < inspva_time)
        {
          RCLCPP_DEBUG(node_.get_logger(), "Discarding oldest CORRIMUDATA.");
          corrimudata_queue_.pop();
          continue;
        }
        else
        {
          RCLCPP_DEBUG(node_.get_logger(), "Discarding oldest INSPVA.");
          inspva_queue_.pop();
          continue;
        }
      }
      // If we've successfully matched up two messages, remove them from their queues.
      inspva_queue_.pop();
      inspvas_queue_.pop();
      corrimudata_queue_.pop();
      corrimus_queue_.pop();

      // Now we can combine them together to make an Imu message.
      auto imu = std::make_shared<sensor_msgs::msg::Imu>();

      imu->header.stamp = corrimudata->header.stamp;

      tf2::Quaternion q;
      q.setRPY(inspva->roll * DEGREES_TO_RADIANS,
               -(inspva->pitch) * DEGREES_TO_RADIANS,
               -(inspva->azimuth) * DEGREES_TO_RADIANS);
      imu->orientation = tf2::toMsg(q);

      if (latest_inscov_)
      {
        imu->orientation_covariance = latest_inscov_->attitude_covariance;
      }
      else if (latest_insstdev_)
      {
        imu->orientation_covariance[0] = std::pow(2, latest_insstdev_->pitch_dev);
        imu->orientation_covariance[4] = std::pow(2, latest_insstdev_->roll_dev);
        imu->orientation_covariance[8] = std::pow(2, latest_insstdev_->azimuth_dev);
      }
      else
      {
        imu->orientation_covariance[0] =
        imu->orientation_covariance[4] =
        imu->orientation_covariance[8] = 1e-3;
      }

      imu->angular_velocity.x = corrimudata->pitch_rate * imu_rate_;
      imu->angular_velocity.y = corrimudata->roll_rate * imu_rate_;
      imu->angular_velocity.z = corrimudata->yaw_rate * imu_rate_;
      imu->angular_velocity_covariance[0] =
      imu->angular_velocity_covariance[4] =
      imu->angular_velocity_covariance[8] = 1e-3;

      imu->linear_acceleration.x = corrimudata->lateral_acceleration * imu_rate_;
      imu->linear_acceleration.y = corrimudata->longitudinal_acceleration * imu_rate_;
      imu->linear_acceleration.z = corrimudata->vertical_acceleration * imu_rate_;
      imu->linear_acceleration_covariance[0] =
      imu->linear_acceleration_covariance[4] =
      imu->linear_acceleration_covariance[8] = 1e-3;

      imu_msgs_.push_back(imu);
    }

    size_t new_size = imu_msgs_.size() - previous_size;
    RCLCPP_DEBUG(node_.get_logger(), "Created %lu new sensor_msgs/Imu messages.", new_size);
  }

  void NovatelGps::SetImuRate(double imu_rate, bool imu_rate_forced)
  {
    RCLCPP_INFO(node_.get_logger(), "IMU sample rate: %f", imu_rate);
    imu_rate_ = imu_rate;
    if (imu_rate_forced)
    {
      imu_rate_forced_ = true;
    }
  }

  void NovatelGps::SetSerialBaud(int32_t serial_baud)
  {
    RCLCPP_INFO(node_.get_logger(), "Serial baud rate : %d", serial_baud);
    serial_baud_ = serial_baud;
  }

  NovatelGps::ReadResult NovatelGps::ParseBinaryMessage(const BinaryMessage& msg,
                                                        const rclcpp::Time& stamp) noexcept(false)
  {
    switch (msg.header_.message_id_)
    {
      case BestposParser::MESSAGE_ID:
      {
        auto position = bestpos_parser_.ParseBinary(msg);
        position->header.stamp = stamp;
        novatel_positions_.push_back(position);
        bestpos_sync_buffer_.push_back(position);
        break;
      }
      case BestxyzParser::MESSAGE_ID:
      {
        auto xyz_position = bestxyz_parser_.ParseBinary(msg);
        xyz_position->header.stamp = stamp;
        novatel_xyz_positions_.push_back(std::move(xyz_position));
        break;
      }
      case BestutmParser::MESSAGE_ID:
      {
        auto utm_position = bestutm_parser_.ParseBinary(msg);
        utm_position->header.stamp = stamp;
        novatel_utm_positions_.push_back(std::move(utm_position));
        break;
      }
      case BestvelParser::MESSAGE_ID:
      {
        auto velocity = bestvel_parser_.ParseBinary(msg);
        velocity->header.stamp = stamp;
        novatel_velocities_.push_back(velocity);
        bestvel_sync_buffer_.push_back(velocity);
        break;
      }
      case Heading2Parser::MESSAGE_ID:
      {
        auto heading = heading2_parser_.ParseBinary(msg);
        heading->header.stamp = stamp;
        heading2_msgs_.push_back(std::move(heading));
        break;
      }
      case DualAntennaHeadingParser::MESSAGE_ID:
      {
        auto heading = dual_antenna_heading_parser_.ParseBinary(msg);
        heading->header.stamp = stamp;
        dual_antenna_heading_msgs_.push_back(std::move(heading));
        break;
      }
      case CorrImuDataParser::MESSAGE_ID:
      {
        auto imu = corrimudata_parser_.ParseBinary(msg);
        imu->header.stamp = stamp;
        corrimudata_msgs_.push_back(imu);
        corrimudata_queue_.push(imu);
        if (corrimudata_queue_.size() > MAX_BUFFER_SIZE)
        {
          // TODO pjr Make this a _THROTTLE log when it's available
          RCLCPP_WARN(node_.get_logger(), "CORRIMUDATA queue overflow.");
          corrimudata_queue_.pop();
        }
        GenerateImuMessages();
        break;
      }
      case CorrImusParser::MESSAGE_ID:
      {
        auto imu = corrimus_parser_.ParseBinary(msg);
        imu->header.stamp = stamp;
        corrimus_msgs_.push_back(imu);
        corrimus_queue_.push(imu);
        if (corrimus_queue_.size() > MAX_BUFFER_SIZE)
        {
          // TODO pjr Make this a _THROTTLE log when it's available
          RCLCPP_WARN(node_.get_logger(), "CORRIMUS queue overflow.");
          corrimus_queue_.pop();
        }
        GenerateImuMessages();
        break;
      }
      case InscovParser::MESSAGE_ID:
      {
        auto inscov = inscov_parser_.ParseBinary(msg);
        inscov->header.stamp = stamp;
        inscov_msgs_.push_back(inscov);
        latest_inscov_ = inscov;
        break;
      }
      case InspvaParser::MESSAGE_ID:
      {
        auto inspva = inspva_parser_.ParseBinary(msg);
        inspva->header.stamp = stamp;
        inspva_msgs_.push_back(inspva);
        inspva_queue_.push(inspva);
        if (inspva_queue_.size() > MAX_BUFFER_SIZE)
        {
          // TODO pjr Make this a _THROTTLE log when it's available
          RCLCPP_WARN(node_.get_logger(), "INSPVA queue overflow.");
          inspva_queue_.pop();
        }
        GenerateImuMessages();
        break;
      }
      case InspvasParser::MESSAGE_ID:
      {
        auto inspvas = inspvas_parser_.ParseBinary(msg);
        inspvas->header.stamp = stamp;
        inspvas_msgs_.push_back(inspvas);
        inspvas_queue_.push(inspvas);
        if (inspvas_queue_.size() > MAX_BUFFER_SIZE)
        {
          // TODO pjr Make this a _THROTTLE log when it's available
          RCLCPP_WARN(node_.get_logger(), "INSPVAS queue overflow.");
          inspvas_queue_.pop();
        }
        GenerateImuMessages();
        break;
      }
      case InspvaxParser::MESSAGE_ID:
      {
        auto inspvax = inspvax_parser_.ParseBinary(msg);
        inspvax->header.stamp = stamp;
        inspvax_msgs_.push_back(std::move(inspvax));
        break;
      }
      case InsstdevParser::MESSAGE_ID:
      {
        auto insstdev = insstdev_parser_.ParseBinary(msg);
        insstdev->header.stamp = stamp;
        insstdev_msgs_.push_back(insstdev);
        latest_insstdev_ = insstdev;
        break;
      }
      case Psrdop2Parser::MESSAGE_ID:
      {
        auto psrdop2 = psrdop2_parser_.ParseBinary(msg);
        psrdop2->header.stamp = stamp;
        psrdop2_msgs_.push_back(psrdop2);
        latest_psrdop2_ = psrdop2;
        break;
      }
      case RangeParser::MESSAGE_ID:
      {
        auto range = range_parser_.ParseBinary(msg);
        range->header.stamp = stamp;
        range_msgs_.push_back(std::move(range));
        break;
      }
      case TimeParser::MESSAGE_ID:
      {
        auto time = time_parser_.ParseBinary(msg);
        utc_offset_ = time->utc_offset;
        RCLCPP_DEBUG(node_.get_logger(), "Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
        time->header.stamp = stamp;
        time_msgs_.push_back(std::move(time));
        break;
      }
      case TrackstatParser::MESSAGE_ID:
      {
        auto trackstat = trackstat_parser_.ParseBinary(msg);
        trackstat->header.stamp = stamp;
        trackstat_msgs_.push_back(std::move(trackstat));
        break;
      }
      case RxStatusParser::MESSAGE_ID:
      {
        auto rxstatus = rxstatus_parser_.ParseBinary(msg);
        rxstatus->header.stamp = stamp;
        rxstatus_msgs_.push_back(std::move(rxstatus));
        break;
      }
      default:
        RCLCPP_WARN(node_.get_logger(), "Unexpected binary message id: %u", msg.header_.message_id_);
        break;
    }

    return READ_SUCCESS;
  }

  NovatelGps::ReadResult NovatelGps::ParseNmeaSentence(const NmeaSentence& sentence,
                                                       const rclcpp::Time& stamp,
                                                       double most_recent_utc_time) noexcept(false)
  {
    if (sentence.id == GpggaParser::MESSAGE_NAME)
    {
      auto gpgga = gpgga_parser_.ParseAscii(sentence);

      auto gpgga_time = UtcFloatToSeconds(gpgga->utc_seconds);

      if (most_recent_utc_time < gpgga_time)
      {
        most_recent_utc_time = gpgga_time;
      }

      gpgga->header.stamp = stamp - std::chrono::duration<double>(most_recent_utc_time - gpgga_time);

      gpgga_msgs_.push_back(std::move(gpgga));
    }
    else if (sentence.id == GprmcParser::MESSAGE_NAME)
    {
      auto gprmc = gprmc_parser_.ParseAscii(sentence);

      auto gprmc_time = UtcFloatToSeconds(gprmc->utc_seconds);

      if (most_recent_utc_time < gprmc_time)
      {
        most_recent_utc_time = gprmc_time;
      }

      gprmc->header.stamp = stamp - std::chrono::duration<double>(most_recent_utc_time - gprmc_time);

      gprmc_msgs_.push_back(std::move(gprmc));
    }
    else if (sentence.id == GpgsaParser::MESSAGE_NAME)
    {
      auto gpgsa = gpgsa_parser_.ParseAscii(sentence);
      gpgsa_msgs_.push_back(std::move(gpgsa));
    }
    else if (sentence.id == GpgsvParser::MESSAGE_NAME)
    {
      auto gpgsv = gpgsv_parser_.ParseAscii(sentence);
      gpgsv_msgs_.push_back(std::move(gpgsv));
    }
    else if (sentence.id == GphdtParser::MESSAGE_NAME)
    {
      auto gphdt = gphdt_parser_.ParseAscii(sentence);
      gphdt_msgs_.push_back(std::move(gphdt));
    }
    else
    {
      RCLCPP_DEBUG(node_.get_logger(), "Unrecognized NMEA sentence %s", sentence.id.c_str());
    }

    return READ_SUCCESS;
  }

  NovatelGps::ReadResult NovatelGps::ParseNovatelSentence(const NovatelSentence& sentence,
                                                          const rclcpp::Time& stamp) noexcept(false)
  {
    if (sentence.id == "BESTPOSA")
    {
      auto position = bestpos_parser_.ParseAscii(sentence);
      position->header.stamp = stamp;
      novatel_positions_.push_back(position);
      bestpos_sync_buffer_.push_back(position);
    }
    else if (sentence.id == "BESTXYZA")
    {
      auto position = bestxyz_parser_.ParseAscii(sentence);
      position->header.stamp = stamp;
      novatel_xyz_positions_.push_back(std::move(position));
    }
    else if (sentence.id == "BESTUTMA")
    {
      auto utm_position = bestutm_parser_.ParseAscii(sentence);
      utm_position->header.stamp = stamp;
      novatel_utm_positions_.push_back(std::move(utm_position));
    }
    else if (sentence.id == "BESTVELA")
    {
      auto velocity = bestvel_parser_.ParseAscii(sentence);
      velocity->header.stamp = stamp;
      novatel_velocities_.push_back(velocity);
      bestvel_sync_buffer_.push_back(velocity);
    }
    else if (sentence.id == "HEADING2A")
    {
      auto heading = heading2_parser_.ParseAscii(sentence);
      heading->header.stamp = stamp;
      heading2_msgs_.push_back(std::move(heading));
    }
    else if (sentence.id == "DUALANTENNAHEADINGA")
    {
      auto heading = dual_antenna_heading_parser_.ParseAscii(sentence);
      heading->header.stamp = stamp;
      dual_antenna_heading_msgs_.push_back(std::move(heading));
    }
    else if (sentence.id == "CORRIMUDATAA")
    {
      auto imu = corrimudata_parser_.ParseAscii(sentence);
      imu->header.stamp = stamp;
      corrimudata_msgs_.push_back(imu);
      corrimudata_queue_.push(imu);
      if (corrimudata_queue_.size() > MAX_BUFFER_SIZE)
      {
        // TODO pjr Make this a _THROTTLE log when it's available
        RCLCPP_WARN(node_.get_logger(), "CORRIMUDATA queue overflow.");
        corrimudata_queue_.pop();
      }
      GenerateImuMessages();
    }
    else if (sentence.id == "INSCOVA")
    {
      auto inscov = inscov_parser_.ParseAscii(sentence);
      inscov->header.stamp = stamp;
      inscov_msgs_.push_back(inscov);
      latest_inscov_ = inscov;
    }
    else if (sentence.id == "INSPVAA")
    {
      auto inspva = inspva_parser_.ParseAscii(sentence);
      inspva->header.stamp = stamp;
      inspva_msgs_.push_back(inspva);
      inspva_queue_.push(inspva);
      if (inspva_queue_.size() > MAX_BUFFER_SIZE)
      {
        // TODO pjr Make this a _THROTTLE log when it's available
        RCLCPP_WARN(node_.get_logger(), "INSPVA queue overflow.");
        inspva_queue_.pop();
      }
      GenerateImuMessages();
    }
    else if (sentence.id == "INSPVAXA")
    {
      auto inspvax = inspvax_parser_.ParseAscii(sentence);
      inspvax->header.stamp = stamp;
      inspvax_msgs_.push_back(std::move(inspvax));
    }
    else if (sentence.id == "INSSTDEVA")
    {
      auto insstdev = insstdev_parser_.ParseAscii(sentence);
      insstdev->header.stamp = stamp;
      insstdev_msgs_.push_back(insstdev);
      latest_insstdev_ = insstdev;
    }
    else if (sentence.id == "PSRDOP2A")
    {
      auto psrdop2 = psrdop2_parser_.ParseAscii(sentence);
      psrdop2->header.stamp = stamp;
      psrdop2_msgs_.push_back(psrdop2);
      latest_psrdop2_ = psrdop2;
    }
    else if (sentence.id == "TIMEA")
    {
      auto time = time_parser_.ParseAscii(sentence);
      utc_offset_ = time->utc_offset;
      RCLCPP_DEBUG(node_.get_logger(), "Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
      time->header.stamp = stamp;
      time_msgs_.push_back(std::move(time));
    }
    else if (sentence.id == "RANGEA")
    {
      auto range = range_parser_.ParseAscii(sentence);
      range->header.stamp = stamp;
      range_msgs_.push_back(std::move(range));
    }
    else if (sentence.id == "TRACKSTATA")
    {
      auto trackstat = trackstat_parser_.ParseAscii(sentence);
      trackstat->header.stamp = stamp;
      trackstat_msgs_.push_back(std::move(trackstat));
    }
    else if (sentence.id == "RXSTATUSA")
    {
      auto rxstatus = rxstatus_parser_.ParseAscii(sentence);
      rxstatus->header.stamp = stamp;
      rxstatus_msgs_.push_back(std::move(rxstatus));
    }
    else if (sentence.id == "RAWIMUXA")
    {
      static std::map<std::string, std::pair<double, std::string>> rates = {
        { "0",  std::pair<double, std::string>(100, "Unknown") },
        { "1",  std::pair<double, std::string>(100, "Honeywell HG1700 AG11") },
        { "4",  std::pair<double, std::string>(100, "Honeywell HG1700 AG17") },
        { "5",  std::pair<double, std::string>(100, "Honeywell HG1700 CA29") },
        { "8",  std::pair<double, std::string>(200, "Litton LN-200 (200hz model)") },
        { "11", std::pair<double, std::string>(100, "Honeywell HG1700 AG58") },
        { "12", std::pair<double, std::string>(100, "Honeywell HG1700 AG62") },
        { "13", std::pair<double, std::string>(200, "iMAR ilMU-FSAS") },
        { "16", std::pair<double, std::string>(200, "KVH 1750 IMU") },
        { "19", std::pair<double, std::string>(200, "Northrop Grumman Litef LCI-1") },
        { "20", std::pair<double, std::string>(100, "Honeywell HG1930 AA99") },
        { "26", std::pair<double, std::string>(100, "Northrop Grumman Litef ISA-100C") },
        { "27", std::pair<double, std::string>(100, "Honeywell HG1900 CA50") },
        { "28", std::pair<double, std::string>(100, "Honeywell HG1930 CA50") },
        { "31", std::pair<double, std::string>(200, "Analog Devices ADIS16488") },
        { "32", std::pair<double, std::string>(125, "Sensonor STIM300") },
        { "33", std::pair<double, std::string>(200, "KVH1750 IMU") },
        { "34", std::pair<double, std::string>(200, "Northrop Grumman Litef ISA-100") },
        { "38", std::pair<double, std::string>(400, "Northrop Grumman Litef ISA-100 400Hz") },
        { "39", std::pair<double, std::string>(400, "Northrop Grumman Litef ISA-100C 400Hz") },
        { "41", std::pair<double, std::string>(125, "Epson G320N") },
        { "45", std::pair<double, std::string>(200, "KVH 1725 IMU?") }, //(This was a guess based on the 1750
                       // as the actual rate is not documented and the specs are similar)
        { "52", std::pair<double, std::string>(200, "Litef microIMU") },
        { "56", std::pair<double, std::string>(125, "Sensonor STIM300, Direct Connection") },
        { "58", std::pair<double, std::string>(200, "Honeywell HG4930 AN01") },
        { "61", std::pair<double, std::string>(100, "Epson G370N") },
       };

      // Parse out the IMU type then save it, we don't care about the rest (3rd field)
      std::string id = sentence.body.size() > 1 ? sentence.body[1] : "";
      if (rates.find(id) != rates.end())
      {
        double rate = rates[id].first;
        RCLCPP_INFO(node_.get_logger(), "IMU Type %s Found, Rate: %f Hz", rates[id].second.c_str(), (float)rate);
        // Set the rate only if it hasn't been forced already
        if (!imu_rate_forced_)
        {
          SetImuRate(rate, false); // Dont force set from here so it can be configured elsewhere
        }
      }
      else
      {
        // Error because the imu type was unknown
        RCLCPP_ERROR(node_.get_logger(), "Unknown IMU Type Received: %s", id.c_str());
      }
    }
    else if (sentence.id == "CLOCKSTEERINGA")
    {
      novatel_gps_driver::ClockSteeringParser::MessageType clocksteering = clocksteering_parser_.ParseAscii(sentence);
      clocksteering_msgs_.push_back(std::move(clocksteering));
    }

    return READ_SUCCESS;
  }

  bool NovatelGps::Write(const std::string& command)
  {
    std::vector<uint8_t> bytes(command.begin(), command.end());

    if (connection_ == SERIAL)
    {
      int32_t written = serial_.Write(bytes);
      if (written != (int32_t)command.length())
      {
        RCLCPP_ERROR(node_.get_logger(), "Failed to send command: %s", command.c_str());
      }
      return written == (int32_t)command.length();
    }
    else if (connection_ == TCP || connection_ == UDP)
    {
      boost::system::error_code error;
      try
      {
        size_t written;
        if (connection_ == TCP)
        {
          written = boost::asio::write(tcp_socket_, boost::asio::buffer(bytes), error);
        }
        else
        {
          written = udp_socket_->send_to(boost::asio::buffer(bytes), *udp_endpoint_, 0, error);
        }
        if (error)
        {
          RCLCPP_ERROR(node_.get_logger(), "Error writing TCP data: %s", error.message().c_str());
          Disconnect();
        }
        RCLCPP_DEBUG(node_.get_logger(), "Wrote %lu bytes.", written);
        return written == (int32_t) command.length();
      }
      catch (std::exception& e)
      {
        Disconnect();
        RCLCPP_ERROR(node_.get_logger(), "Exception writing TCP data: %s", e.what());
      }
    }
    else if (connection_ == PCAP)
    {
      RCLCPP_WARN_ONCE(node_.get_logger(), "Writing data is unsupported in pcap mode.");
      return true;
    }

    return false;
  }

  bool NovatelGps::Configure(NovatelMessageOpts const& opts)
  {
    bool configured = true;
    configured = configured && Write("unlogall THISPORT_ALL\r\n");

    if (apply_vehicle_body_rotation_)
    {
      configured = configured && Write("vehiclebodyrotation 0 0 90\r\n");
      configured = configured && Write("applyvehiclebodyrotation\r\n");
    }

    for(const auto& option : opts)
    {
      std::stringstream command;
      command << std::setprecision(3);
      if (option.first.find("heading2") != std::string::npos)
      {
      	command << "log " << option.first << " onnew " << "\r\n";
      }
      else if (option.second < 0.0)
      {
        command << "log " << option.first << " onchanged\r\n";
      }
      else
      {
      	command << "log " << option.first << " ontime " << option.second << "\r\n";
      }
      configured = configured && Write(command.str());
    }

    // Log the IMU data once to get the IMU type
    configured = configured && Write("log rawimuxa\r\n");

    return configured;
  }
}
