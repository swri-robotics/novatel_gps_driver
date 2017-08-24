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

#include <sstream>
#include <novatel_gps_driver/novatel_gps.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>

namespace novatel_gps_driver
{
  NovatelGps::NovatelGps() :
      gpgga_gprmc_sync_tol_(0.01),
      gpgga_position_sync_tol_(0.01),
      wait_for_position_(false),
      connection_(SERIAL),
      is_connected_(false),
      utc_offset_(0),
      tcp_socket_(io_service_),
      gpgga_msgs_(MAX_BUFFER_SIZE),
      gpgga_sync_buffer_(SYNC_BUFFER_SIZE),
      gpgsa_msgs_(MAX_BUFFER_SIZE),
      gpgsv_msgs_(MAX_BUFFER_SIZE),
      gprmc_msgs_(MAX_BUFFER_SIZE),
      gprmc_sync_buffer_(SYNC_BUFFER_SIZE),
      novatel_positions_(MAX_BUFFER_SIZE),
      novatel_velocities_(MAX_BUFFER_SIZE),
      position_sync_buffer_(SYNC_BUFFER_SIZE),
      range_msgs_(MAX_BUFFER_SIZE),
      time_msgs_(MAX_BUFFER_SIZE),
      trackstat_msgs_(MAX_BUFFER_SIZE)
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
    is_connected_ = false;
  }

  NovatelGps::ReadResult NovatelGps::ProcessData()
  {
    NovatelGps::ReadResult read_result = ReadData();

    if (read_result != READ_SUCCESS)
    {
      return read_result;
    }

    ros::Time stamp = ros::Time::now();
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

      ROS_DEBUG("Parsed: %lu NMEA / %lu NovAtel / %lu Binary messages",
               nmea_sentences.size(), novatel_sentences.size(), binary_messages.size());
      if (!nmea_buffer_.empty())
      {
        ROS_DEBUG("%lu unparsed bytes left over.", nmea_buffer_.size());
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
        ROS_WARN("%s", p.what());
        ROS_WARN("For sentence: [%s]", boost::algorithm::join(sentence.body, ",").c_str());
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
        ROS_WARN("%s", p.what());
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
        ROS_WARN("%s", p.what());
        read_result = READ_PARSE_FAILED;
      }
    }

    return read_result;
  }

  void NovatelGps::GetNovatelPositions(std::vector<novatel_gps_msgs::NovatelPositionPtr>& positions)
  {
    positions.clear();
    positions.insert(positions.end(), novatel_positions_.begin(), novatel_positions_.end());
    novatel_positions_.clear();
  }

  void NovatelGps::GetNovatelVelocities(std::vector<novatel_gps_msgs::NovatelVelocityPtr>& velocities)
  {
    velocities.resize(novatel_velocities_.size());
    std::copy(novatel_velocities_.begin(), novatel_velocities_.end(), velocities.begin());
    novatel_velocities_.clear();
  }

  void NovatelGps::GetFixMessages(std::vector<gps_common::GPSFixPtr>& fix_messages)
  {
    // Clear out the fix_messages list before filling it
    fix_messages.clear();

    // both a gpgga and a gprmc message are required to fill the GPSFix message
    while (!gpgga_sync_buffer_.empty() && !gprmc_sync_buffer_.empty())
    {
      double gpgga_time = gpgga_sync_buffer_.front().utc_seconds;
      double gprmc_time = gprmc_sync_buffer_.front().utc_seconds;

      // Find the time difference between gpgga and gprmc time
      double dt = gpgga_time - gprmc_time;
      // Handle times around midnight
      if (dt > 43200.0)
      {
        dt -= 86400.0;
      }
      if (dt < -43200.0)
      {
        dt += 86400.0;
      }

      // Get the front elements of the gpgga and gprmc buffers synced to within tolerance
      if (dt > gpgga_gprmc_sync_tol_)
      {
        // The gprmc message is more than tol older than the gpgga message,
        // discard it and continue
        gprmc_sync_buffer_.pop_front();
      }
      else if (-dt > gpgga_gprmc_sync_tol_)
      {
        // The gpgga message is more than tol older than the gprmc message,
        // discard it and continue
        gpgga_sync_buffer_.pop_front();
      }
      else // The gpgga and gprmc messages are synced
      {
        bool has_position = false;
        bool pop_position = false;

        // Iterate over the position message buffer until we find one
        // that is synced with the gpgga message
        while (!position_sync_buffer_.empty())
        {
          // Calculate UTC position time from GPS seconds by subtracting out
          // full days and applying the UTC offset
          double gps_seconds = position_sync_buffer_.front()->novatel_msg_header.gps_seconds + utc_offset_;
          if (gps_seconds < 0)
          {
              // Handle times around week boundaries
              gps_seconds = gps_seconds + 604800;  // 604800 = 7 * 24 * 60 * 60 (seconds in a week)
          }
          int32_t days = static_cast<int32_t>(gps_seconds / 86400.0);
          double position_time = gps_seconds - days * 86400.0;

          // Find the time difference between gpgga and position time
          double pos_dt = gpgga_time - position_time;
          // Handle times around midnight
          if (pos_dt > 43200.0)
          {
            pos_dt -= 86400.0;
          }
          if (pos_dt < -43200.0)
          {
            pos_dt += 86400.0;
          }
          if (pos_dt > gpgga_position_sync_tol_)
          {
            // The position message is more than tol older than the gpgga message,
            // discard it and continue
            ROS_DEBUG("Discarding a position message that is too old (%f < %f)", position_time, gpgga_time);
            position_sync_buffer_.pop_front();
          }
          else if (-pos_dt > gpgga_position_sync_tol_)
          {
            // The position message is more than tol ahead of the gpgga message,
            // use it but don't pop it
            ROS_DEBUG("Waiting because the most recent GPGGA message is too old (%f > %f)", position_time, gpgga_time);
            has_position = true;
            break;
          }
          else //the gpgga and position tol messages are in sync
          {
            has_position = true;
            pop_position = true;
            break;
          }
        }

        if (has_position || !wait_for_position_)
        {
          // If we have a position message (or don't need one), create and fill
          // a GPS fix message
          gps_common::GPSFixPtr gps_fix = boost::make_shared<gps_common::GPSFix>();
          // Fill GPS fix message using the messages at the front of the two
          // sync queues
          extractor_.GetGpsFixMessage(
              gprmc_sync_buffer_.front(),
              gpgga_sync_buffer_.front(),
              gps_fix);

          // Remove the used messages from the two sync queues
          gpgga_sync_buffer_.pop_front();
          gprmc_sync_buffer_.pop_front();

          if (has_position)
          {
            // We have a position message, so we can calculate variances
            // from the standard deviations
            double sigma_x = position_sync_buffer_.front()->lon_sigma;
            gps_fix->position_covariance[0] = sigma_x * sigma_x;

            double sigma_y = position_sync_buffer_.front()->lat_sigma;
            gps_fix->position_covariance[4] = sigma_y * sigma_y;

            double sigma_z = position_sync_buffer_.front()->height_sigma;
            gps_fix->position_covariance[8] = sigma_z * sigma_z;

            gps_fix->position_covariance_type =
                gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            if (pop_position)
            {
              position_sync_buffer_.pop_front();
            }
          }

          // Add the message to the fix message list
          fix_messages.push_back(gps_fix);
        }
        else  // There is no position message (and wait_for_position is true)
        {
          // return without pushing any more gps fix messages to the list
          return;
        }
      }  // else (gpgga and gprmc synced)
    }  // while (gpgga and gprmc buffers contain messages)
  }

  void NovatelGps::GetNovatelCorrectedImuData(std::vector<novatel_gps_msgs::NovatelCorrectedImuDataPtr>& imu_messages)
  {
    imu_messages.clear();
    imu_messages.insert(imu_messages.end(), imu_messages_.begin(), imu_messages_.end());
    imu_messages_.clear();
  }

  void NovatelGps::GetGpggaMessages(std::vector<novatel_gps_msgs::GpggaPtr>& gpgga_messages)
  {
    gpgga_messages.clear();
    gpgga_messages.insert(gpgga_messages.end(), gpgga_msgs_.begin(), gpgga_msgs_.end());
    gpgga_msgs_.clear();
  }

  void NovatelGps::GetGpgsaMessages(std::vector<novatel_gps_msgs::GpgsaPtr>& gpgsa_messages)
  {
    gpgsa_messages.resize(gpgsa_msgs_.size());
    std::copy(gpgsa_msgs_.begin(), gpgsa_msgs_.end(), gpgsa_messages.begin());
    gpgsa_msgs_.clear();
  }

  void NovatelGps::GetGpgsvMessages(std::vector<novatel_gps_msgs::GpgsvPtr>& gpgsv_messages)
  {
    gpgsv_messages.resize(gpgsv_msgs_.size());
    std::copy(gpgsv_msgs_.begin(), gpgsv_msgs_.end(), gpgsv_messages.begin());
    gpgsv_msgs_.clear();
  }

  void NovatelGps::GetGprmcMessages(std::vector<novatel_gps_msgs::GprmcPtr>& gprmc_messages)
  {
    gprmc_messages.clear();
    gprmc_messages.insert(gprmc_messages.end(), gprmc_msgs_.begin(), gprmc_msgs_.end());
    gprmc_msgs_.clear();
  }
  
  void NovatelGps::GetRangeMessages(std::vector<novatel_gps_msgs::RangePtr>& range_messages)
  {
    range_messages.resize(range_msgs_.size());
    std::copy(range_msgs_.begin(), range_msgs_.end(), range_messages.begin());
    range_msgs_.clear();
  }

  void NovatelGps::GetTimeMessages(std::vector<novatel_gps_msgs::TimePtr>& time_messages)
  {
    time_messages.resize(time_msgs_.size());
    std::copy(time_msgs_.begin(), time_msgs_.end(), time_messages.begin());
    time_msgs_.clear();
  }

  void NovatelGps::GetTrackstatMessages(std::vector<novatel_gps_msgs::TrackstatPtr>& trackstat_msgs)
  {
    trackstat_msgs.resize(trackstat_msgs_.size());
    std::copy(trackstat_msgs_.begin(), trackstat_msgs_.end(), trackstat_msgs.begin());
    trackstat_msgs_.clear();
  }

  bool NovatelGps::CreateSerialConnection(const std::string& device, NovatelMessageOpts const& opts)
  {
    swri_serial_util::SerialConfig config;
    config.baud = 115200;
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
        ROS_ERROR("Failed to configure GPS. This port may be read only, or the "
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
      ROS_INFO("Using default port.");
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
          ROS_INFO("Connecting via TCP to %s:%s", ip.c_str(), port.c_str());
        }
        else
        {
          boost::asio::ip::udp::resolver resolver(io_service_);
          boost::asio::ip::udp::resolver::query query(ip, port);
          udp_endpoint_ = boost::make_shared<boost::asio::ip::udp::endpoint>(*resolver.resolve(query));
          udp_socket_.reset(new boost::asio::ip::udp::socket(io_service_));
          udp_socket_->open(boost::asio::ip::udp::v4());
          ROS_INFO("Connecting via UDP to %s:%s", ip.c_str(), port.c_str());
        }
      }
      else
      {
        uint16_t port_num = static_cast<uint16_t>(strtoll(port.c_str(), NULL, 10));
        if (connection_ == TCP)
        {
          boost::asio::ip::tcp::acceptor acceptor(io_service_,
                                                  boost::asio::ip::tcp::endpoint(
                                                      boost::asio::ip::tcp::v4(), port_num));
          ROS_INFO("Listening on TCP port %s", port.c_str());
          acceptor.accept(tcp_socket_);
          ROS_INFO("Accepted TCP connection from client: %s",
                   tcp_socket_.remote_endpoint().address().to_string().c_str());
        }
        else
        {
          udp_socket_.reset(new boost::asio::ip::udp::socket(
              io_service_,
              boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                             port_num)));
          boost::array<char, 1> recv_buf;
          udp_endpoint_ = boost::make_shared<boost::asio::ip::udp::endpoint>();
          boost::system::error_code error;

          ROS_INFO("Listening on UDP port %s", port.c_str());
          udp_socket_->receive_from(boost::asio::buffer(recv_buf), *udp_endpoint_, 0, error);
          if (error && error != boost::asio::error::message_size)
          {
            throw boost::system::system_error(error);
          }

          ROS_INFO("Accepted UDP connection from client: %s",
                   udp_endpoint_->address().to_string().c_str());
        }
      }
    }
    catch (std::exception& e)
    {
      error_msg_ = e.what();
      ROS_ERROR("Unable to connect: %s", e.what());
      return false;
    }

    is_connected_ = true;

    if (Configure(opts))
    {
      ROS_INFO("Configured GPS.");
    }
    else
    {
      // We will not kill the connection here, because the device may already
      // be setup to communicate correctly, but we will print a warning
      ROS_ERROR("Failed to configure GPS. This port may be read only, or the "
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
        ROS_WARN("TCP connection error: %s", e.what());
      }
    }

    error_msg_ = "Unsupported connection type.";

    return READ_ERROR;
  }

  NovatelGps::ReadResult NovatelGps::ParseBinaryMessage(const BinaryMessage& msg,
                                                        const ros::Time& stamp) throw(ParseException)
  {
    switch (msg.header_.message_id_)
    {
      case BestposParser::MESSAGE_ID:
      {
        novatel_gps_msgs::NovatelPositionPtr position = bestpos_parser_.ParseBinary(msg);
        position->header.stamp = stamp;
        novatel_positions_.push_back(position);
        position_sync_buffer_.push_back(position);
        break;
      }
      case BestvelParser::MESSAGE_ID:
      {
        novatel_gps_msgs::NovatelVelocityPtr velocity = bestvel_parser_.ParseBinary(msg);
        velocity->header.stamp = stamp;
        novatel_velocities_.push_back(velocity);
        break;
      }
      case CorrImuDataParser::MESSAGE_ID:
      {
        novatel_gps_msgs::NovatelCorrectedImuDataPtr imu = corrimudata_parser_.ParseBinary(msg);
        imu->header.stamp = stamp;
        imu_messages_.push_back(imu);
        break;
      }
      case RangeParser::MESSAGE_ID:
      {
        novatel_gps_msgs::RangePtr range = range_parser_.ParseBinary(msg);
        range->header.stamp = stamp;
        range_msgs_.push_back(range);
        break;
      }
      case TimeParser::MESSAGE_ID:
      {
        novatel_gps_msgs::TimePtr time = time_parser_.ParseBinary(msg);
        utc_offset_ = time->utc_offset;
        ROS_DEBUG("Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
        time->header.stamp = stamp;
        time_msgs_.push_back(time);
        break;
      }
      case TrackstatParser::MESSAGE_ID:
      {
        novatel_gps_msgs::TrackstatPtr trackstat = trackstat_parser_.ParseBinary(msg);
        trackstat->header.stamp = stamp;
        trackstat_msgs_.push_back(trackstat);
        break;
      }
      default:
        ROS_WARN("Unexpected binary message id: %u", msg.header_.message_id_);
        break;
    }

    return READ_SUCCESS;
  }

  NovatelGps::ReadResult NovatelGps::ParseNmeaSentence(const NmeaSentence& sentence,
                                                       const ros::Time& stamp,
                                                       double most_recent_utc_time) throw(ParseException)
  {
    if (sentence.id == GpggaParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GpggaPtr gpgga = gpgga_parser_.ParseAscii(sentence);

      if (most_recent_utc_time < gpgga->utc_seconds)
      {
        most_recent_utc_time = gpgga->utc_seconds;
      }

      gpgga->header.stamp = stamp - ros::Duration(most_recent_utc_time - gpgga->utc_seconds);

      if (gpgga_parser_.WasLastGpsValid())
      {
        gpgga_msgs_.push_back(gpgga);

        // Make a deep copy for the sync buffer so the GPSFix messages
        // don't get adjusted multiple times for the sync offset.
        gpgga_sync_buffer_.push_back(*gpgga);
      }
      else
      {
        gpgga_msgs_.push_back(gpgga);
      }
    }
    else if (sentence.id == GprmcParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GprmcPtr gprmc = gprmc_parser_.ParseAscii(sentence);

      if (most_recent_utc_time < gprmc->utc_seconds)
      {
        most_recent_utc_time = gprmc->utc_seconds;
      }

      gprmc->header.stamp = stamp - ros::Duration(most_recent_utc_time - gprmc->utc_seconds);

      if (gprmc_parser_.WasLastGpsValid())
      {
        gprmc_msgs_.push_back(gprmc);

        // Make a deep copy for the sync buffer so the GPSFix messages
        // don't get adjusted multiple times for the sync offset.
        gprmc_sync_buffer_.push_back(*gprmc);
      }
      else
      {
        gprmc_msgs_.push_back(gprmc);
      }
    }
    else if (sentence.id == GpgsaParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GpgsaPtr gpgsa = gpgsa_parser_.ParseAscii(sentence);
      gpgsa_msgs_.push_back(gpgsa);
    }
    else if (sentence.id == GpgsvParser::MESSAGE_NAME)
    {
      novatel_gps_msgs::GpgsvPtr gpgsv = gpgsv_parser_.ParseAscii(sentence);
      gpgsv_msgs_.push_back(gpgsv);
    }
    else
    {
      ROS_DEBUG_STREAM("Unrecognized NMEA sentence " << sentence.id);
    }

    return READ_SUCCESS;
  }

  NovatelGps::ReadResult NovatelGps::ParseNovatelSentence(const NovatelSentence& sentence,
                                                          const ros::Time& stamp) throw(ParseException)
  {
    if (sentence.id == "BESTPOSA")
    {
      novatel_gps_msgs::NovatelPositionPtr position = bestpos_parser_.ParseAscii(sentence);
      position->header.stamp = stamp;
      novatel_positions_.push_back(position);
      position_sync_buffer_.push_back(position);
    }
    else if (sentence.id == "BESTVELA")
    {
      novatel_gps_msgs::NovatelVelocityPtr velocity = bestvel_parser_.ParseAscii(sentence);
      velocity->header.stamp = stamp;
      novatel_velocities_.push_back(velocity);
    }
    else if (sentence.id == "CORRIMUDATAA")
    {
      novatel_gps_msgs::NovatelCorrectedImuDataPtr imu = corrimudata_parser_.ParseAscii(sentence);
      imu->header.stamp = stamp;
      imu_messages_.push_back(imu);
    }
    else if (sentence.id == "TIMEA")
    {
      novatel_gps_msgs::TimePtr time = time_parser_.ParseAscii(sentence);
      utc_offset_ = time->utc_offset;
      ROS_DEBUG("Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
      time->header.stamp = stamp;
      time_msgs_.push_back(time);
    }
    else if (sentence.id == "RANGEA")
    {
      novatel_gps_msgs::RangePtr range = range_parser_.ParseAscii(sentence);
      range->header.stamp = stamp;
      range_msgs_.push_back(range);
    }
    else if (sentence.id == "TRACKSTATA")
    {
      novatel_gps_msgs::TrackstatPtr trackstat = trackstat_parser_.ParseAscii(sentence);
      trackstat->header.stamp = stamp;
      trackstat_msgs_.push_back(trackstat);
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
        ROS_ERROR("Failed to send command: %s", command.c_str());
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
          ROS_ERROR("Error writing TCP data: %s", error.message().c_str());
          Disconnect();
        }
        ROS_DEBUG("Wrote %lu bytes.", written);
        return written == (int32_t) command.length();
      }
      catch (std::exception& e)
      {
        Disconnect();
        ROS_ERROR("Exception writing TCP data: %s", e.what());
      }
    }

    return false;
  }

  bool NovatelGps::Configure(NovatelMessageOpts const& opts)
  {
    bool configured = true;
    configured = configured && Write("unlogall true\n");
    for(NovatelMessageOpts::const_iterator option = opts.begin(); option != opts.end(); ++option)
    {
      std::stringstream command;
      command << std::setprecision(3);
      command << "log " << option->first << " ontime " << option->second << "\n";
      configured = configured && Write(command.str());
    }
    return configured;
  }
}
