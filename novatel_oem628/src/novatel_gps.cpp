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

#include <novatel_oem628/novatel_gps.h>

#include <boost/make_shared.hpp>

#include <novatel_oem628/novatel_message_parser.h>

namespace novatel_oem628
{
  NovatelGps::NovatelGps() :
      connection_(SERIAL),
      utc_offset_(0),
      tcp_socket_(io_service_),
      udp_socket_(io_service_),
      novatel_positions_(100),
      gpgga_msgs_(100),
      gprmc_msgs_(100),
      gpgga_sync_buffer_(10),
      gprmc_sync_buffer_(10),
      position_sync_buffer_(10)
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
    Disconnect();

    connection_ = connection;

    if (connection_ == SERIAL)
    {
      return CreateSerialConnection(device);
    }
    else if (connection_ == TCP)
    {
      return CreateTcpConnection(device);
    }
    else if (connection_ == UDP)
    {
      return CreateUdpConnection(device);
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
      udp_socket_.close();
    }
  }

  NovatelGps::ReadResult NovatelGps::ProcessData()
  {
    NovatelGps::ReadResult read_result = ReadData();

    if (read_result != READ_SUCCESS)
    {
      return read_result;
    }

    ros::Time stamp = ros::Time::now();

    if (!data_buffer_.empty())
    {
      nmea_buffer_.append(
        (char*)&data_buffer_[0],
        data_buffer_.size());

      data_buffer_.clear();

      if (!extract_complete_sentences(
          nmea_buffer_,
          nmea_sentences_,
          novatel_sentences_,
          nmea_buffer_))
      {
        read_result = READ_PARSE_FAILED;
        error_msg_ = "Parse failure extracting NMEA sentences.";
      }
    }

    double most_recent_utc_time = GetMostRecentUtcTime(nmea_sentences_);

    for (size_t i = 0; i < nmea_sentences_.size(); i++)
    {
      if (nmea_sentences_[i].id == "GPGGA")
      {
        GpggaPtr gpgga = boost::make_shared<Gpgga>();
        NmeaMessageParseResult parse_result =
          parse_vectorized_gpgga_message(nmea_sentences_[i].body, gpgga);

        if (most_recent_utc_time < gpgga->utc_seconds)
        {
          most_recent_utc_time = gpgga->utc_seconds;
        }

        gpgga->header.stamp = stamp - ros::Duration(most_recent_utc_time - gpgga->utc_seconds);

        if (parse_result == ParseSucceededAndGpsDataValid)
        {
          gpgga_msgs_.push_back(gpgga);

          // Make a deep copy for the sync buffer so the GPSFix messages
          // don't get adjusted multiple times for the sync offset.
          gpgga_sync_buffer_.push_back(*gpgga);
        }
        else if (parse_result == ParseSucceededAndGpsDataNotValid)
        {
          gpgga_msgs_.push_back(gpgga);
        }
        else
        {
          read_result = READ_PARSE_FAILED;
          error_msg_ = "Failed to parse the NMEA GPGGA message.";
        }
      }
      else if (nmea_sentences_[i].id == "GPRMC")
      {
        GprmcPtr gprmc = boost::make_shared<Gprmc>();
        NmeaMessageParseResult parse_result =
          parse_vectorized_gprmc_message(nmea_sentences_[i].body, gprmc);

        if (most_recent_utc_time < gprmc->utc_seconds)
        {
          most_recent_utc_time = gprmc->utc_seconds;
        }

        gprmc->header.stamp = stamp - ros::Duration(most_recent_utc_time - gprmc->utc_seconds);

        if (parse_result == ParseSucceededAndGpsDataValid)
        {
          gprmc_msgs_.push_back(gprmc);

          // Make a deep copy for the sync buffer so the GPSFix messages
          // don't get adjusted multiple times for the sync offset.
          gprmc_sync_buffer_.push_back(*gprmc);
        }
        else if (parse_result == ParseSucceededAndGpsDataNotValid)
        {
          gprmc_msgs_.push_back(gprmc);
        }
        else
        {
          read_result = READ_PARSE_FAILED;
          error_msg_ = "Failed to parse the NMEA GPRMC message.";
        }
      }
    }
    nmea_sentences_.clear();

    for (size_t i = 0; i < novatel_sentences_.size(); i++)
    {
      if (novatel_sentences_[i].id == "BESTPOSA")
      {
        NovatelPositionPtr position = boost::make_shared<NovatelPosition>();
        if (!parse_novatel_pos_msg(novatel_sentences_[i], position))
        {
          read_result = READ_PARSE_FAILED;
          error_msg_ = "Failed to parse the Novatel BestPos message.";
        }
        else
        {
          position->header.stamp = stamp;
          novatel_positions_.push_back(position);
          position_sync_buffer_.push_back(position);
        }
      }
      else if (novatel_sentences_[i].id == "TIMEA")
      {
        double offset = 0;
        if (!ParseNovatelTimeMessage(novatel_sentences_[i], offset))
        {
          read_result = READ_PARSE_FAILED;
          error_msg_ = "Failed to parse the Novatel Time message.";
        }
        else
        {
          utc_offset_ = offset;
        }
      }
    }
    novatel_sentences_.clear();

    return read_result;
  }

  void NovatelGps::GetNovatelPositions(std::vector<NovatelPositionPtr>& positions)
  {
    positions.clear();
    positions.insert(positions.end(), novatel_positions_.begin(), novatel_positions_.end());
    novatel_positions_.clear();
  }

  void NovatelGps::GetFixMessages(std::vector<gps_common::GPSFixPtr>& fix_messages)
  {
    // Clear out the fix_messages list before filling it
    fix_messages.clear();

    // We need both a gpgga and a gprmc message to fill the GPSFix message
    if (gpgga_sync_buffer_.empty() || gprmc_sync_buffer_.empty())
    {
      return;
    }

    // Loop through all the messages in the buffer, using them to fill GPSFix
    // messages
    while (!gpgga_sync_buffer_.empty() && !gprmc_sync_buffer_.empty())
    {
      double gpgga_time = gpgga_sync_buffer_.front().utc_seconds;
      double gprmc_time = gprmc_sync_buffer_.front().utc_seconds;

      // Check if the gppga time at the front of the queue is synced with the
      // gprmc time at the front of the queue to within 10 ms
      if (gpgga_time - gprmc_time > 0.01)
      {
        // The gprmc message is more then 10ms older than the gppga message,
        // so discard it and try again
        gprmc_sync_buffer_.pop_front();
      }
      else if (gprmc_time - gpgga_time > 0.01)
      {
        // The gppga message is more than 10ms older than the gprmc message, so
        // discard it and try again
        gpgga_sync_buffer_.pop_front();
      }
      else  // The gppga and gprmc messages are synced to within 10 ms
      {
        // If there are no messages left in either buffer, we do not need to
        // wait for a position message
        bool wait_for_position = true;
        if (gpgga_sync_buffer_.size() > 0 && gprmc_sync_buffer_.size() > 0)
        {
          wait_for_position = false;
        }

        double position_time = 0;
        // Iterate over the position synce message buffer until we find one
        // that is within 10ms of the gppga message
        while (!position_sync_buffer_.empty() && gpgga_time - position_time > 0.01)
        {
          // Calculate UTC position time from GPS seconds by subtracting out
          // full days and applying the UTC offset
          // TODO(evenator): Should UTC offset be applied *before* calculating
          //   days?
          double gps_seconds = position_sync_buffer_.front()->novatel_msg_header.gps_seconds;
          int32_t days = gps_seconds / 86400.0;
          position_time = gps_seconds - days * 86400.0 + utc_offset_;

          // If the message is more than 10 ms behind the gpgga time, discard
          // it and continue
          if (gpgga_time - position_time > 0.01)
          {
            position_sync_buffer_.pop_front();
          }
          else  // The position and gppga messages are synced to within 10 ms
          {
            break;  // All 3 messages are synced, so break the loop
          }
        }

        // If a synced position message was found, set has_position true
        bool has_position = std::fabs(gpgga_time - position_time) < 0.01;

        if (has_position || !wait_for_position)
        {
          // If we have a position message (or don't need one), create and fill
          // a GPS fix message
          gps_common::GPSFixPtr gps_fix = boost::make_shared<gps_common::GPSFix>();
          // Fill GPS fix message using the messages at the front of the two
          // sync queues
          get_gps_fix_message(
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

            position_sync_buffer_.pop_front();
          }

          // Add the message to the fix message list
          fix_messages.push_back(gps_fix);
        }
        else  // There is no position message (and wait_for_position is true)
        {
          // We're out of position messages, so return without pushing any
          // more messages to the list
          return;
        }
      }  // else (gppga and gprmc synced)
    }  // while (gppga queue and gprm queue contain messages)
  }

  void NovatelGps::GetGpggaMessages(std::vector<GpggaPtr>& gpgga_messages)
  {
    gpgga_messages.clear();
    gpgga_messages.insert(gpgga_messages.end(), gpgga_msgs_.begin(), gpgga_msgs_.end());
    gpgga_msgs_.clear();
  }

  void NovatelGps::GetGprmcMessages(std::vector<GprmcPtr>& gprmc_messages)
  {
    gprmc_messages.clear();
    gprmc_messages.insert(gprmc_messages.end(), gprmc_msgs_.begin(), gprmc_msgs_.end());
    gprmc_msgs_.clear();
  }

  bool NovatelGps::CreateSerialConnection(const std::string& device)
  {
    serial_util::SerialConfig config;
    config.baud = 115200;
    config.parity = serial_util::SerialConfig::NO_PARITY;
    config.flow_control = false;
    config.data_bits = 8;
    config.stop_bits = 1;
    config.low_latency_mode = false;

    if (device.find("ttyUSB") != std::string::npos ||
        device.find("ttyAMC") != std::string::npos)
    {
      config.writable = true;
    }

    bool success = serial_.Open(device, config);

    if (!success)
    {
      error_msg_ = serial_.ErrorMsg();
    }

    if (success && config.writable)
    {
      if (!Configure())
      {
        ROS_ERROR("Failed to configure GPS");
        serial_.Close();
        return false;
      }
    }

    return success;
  }

  bool NovatelGps::CreateTcpConnection(const std::string& device)
  {
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(device);
    boost::system::error_code error_code = boost::asio::error::host_not_found;

    // TODO(malban)

    return false;
  }

  bool NovatelGps::CreateUdpConnection(const std::string& device)
  {
    // TODO(malban)

    return false;
  }

  NovatelGps::ReadResult NovatelGps::ReadData()
  {
    if (connection_ == SERIAL)
    {
      serial_util::SerialPort::Result result =
          serial_.ReadBytes(data_buffer_, 0, 1000);

      if (result == serial_util::SerialPort::ERROR)
      {
        error_msg_ = serial_.ErrorMsg();
        return READ_ERROR;
      }
      else if (result == serial_util::SerialPort::TIMEOUT)
      {
        error_msg_ = "Timed out waiting for serial device.";
        return READ_TIMEOUT;
      }
      else if (result == serial_util::SerialPort::INTERRUPTED)
      {
        error_msg_ = "Interrupted during read from serial device.";
        return READ_INTERRUPTED;
      }

      return READ_SUCCESS;
    }

    // TODO(malban)

    error_msg_ = "unsupported connection type.";
    return READ_ERROR;
  }

  bool NovatelGps::Write(const std::string& command)
  {
    std::vector<uint8_t> bytes(command.length());
    for (size_t i = 0; i < command.length(); i++)
    {
      bytes[i] = command[i];
    }

    if (connection_ == SERIAL)
    {
      int32_t written = serial_.Write(bytes);
      if (written != (int32_t)command.length())
      {
        ROS_ERROR("Failed to send command: %s", command.c_str());
      }
      return written == (int32_t)command.length();
    }

    return false;
  }

  bool NovatelGps::Configure()
  {
    bool configured = true;
    configured = configured && Write("unlogall\n");
    configured = configured && Write("log gpgga ontime 0.05\n");
    configured = configured && Write("log gprmc ontime 0.05\n");
    configured = configured && Write("log bestposa ontime 0.05\n");
    configured = configured && Write("log timea ontime 1.0\n");
    return configured;
  }
}
