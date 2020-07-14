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

#include <novatel_gps_driver/nodes/novatel_gps_node.h>

#include <swri_math_util/math_util.h>

#include <swri_roscpp/publisher.h>

#include <rcl/time.h>

namespace stats = boost::accumulators;

namespace novatel_gps_driver
{
  NovatelGpsNode::NovatelGpsNode(const rclcpp::NodeOptions& options) :
      rclcpp::Node("novatel_gps", options),
      device_(""),
      connection_type_("serial"),
      serial_baud_(115200),
      polling_period_(0.05),
      publish_gpgsa_(false),
      publish_gpgsv_(false),
      publish_gphdt_(false),
      imu_rate_(100.0),
      imu_sample_rate_(-1),
      span_frame_to_ros_frame_(false),
      publish_clock_steering_(false),
      publish_imu_messages_(false),
      publish_novatel_positions_(false),
      publish_novatel_xyz_positions_(false),
      publish_novatel_utm_positions_(false),
      publish_novatel_velocity_(true),
      publish_novatel_heading2_(false),
      publish_novatel_dual_antenna_heading_(false),
      publish_novatel_psrdop2_(false),
      publish_nmea_messages_(false),
      publish_range_messages_(false),
      publish_time_messages_(false),
      publish_trackstat_(false),
      publish_diagnostics_(true),
      publish_sync_diagnostic_(true),
      publish_invalid_gpsfix_(false),
      reconnect_delay_s_(0.5),
      use_binary_messages_(false),
      connection_(NovatelGps::SERIAL),
      gps_(*this),
      last_sync_(get_clock()->get_clock_type()),
      rolling_offset_(stats::tag::rolling_window::window_size = 10),
      diagnostic_updater_(this),
      expected_rate_(20),
      device_timeouts_(0),
      device_interrupts_(0),
      device_errors_(0),
      gps_parse_failures_(0),
      gps_insufficient_data_warnings_(0),
      publish_rate_warnings_(0),
      measurement_count_(0),
      last_published_(get_clock()->get_clock_type()),
      imu_frame_id_(""),
      frame_id_("")
  {
    /**
     * Init method reads parameters and sets up publishers and subscribers.
     * It does not connect to the device.
     */
    RCLCPP_INFO(this->get_logger(), "Initializing...");
    device_ = this->declare_parameter("device", device_);
    imu_rate_ = this->declare_parameter("imu_rate", imu_rate_);
    imu_sample_rate_ = this->declare_parameter("imu_sample_rate", imu_sample_rate_);
    publish_clock_steering_ = this->declare_parameter("publish_clocksteering", publish_clock_steering_);
    publish_gpgsa_ = this->declare_parameter("publish_gpgsa", publish_gpgsa_);
    publish_gpgsv_ = this->declare_parameter("publish_gpgsv", publish_gpgsv_);
    publish_gphdt_ = this->declare_parameter("publish_gphdt", publish_gphdt_);
    publish_imu_messages_ = this->declare_parameter("publish_imu_messages", publish_imu_messages_);
    publish_invalid_gpsfix_ = this->declare_parameter("publish_invalid_gpsfix", false);
    publish_novatel_positions_ = this->declare_parameter("publish_novatel_positions", publish_novatel_positions_);
    publish_novatel_xyz_positions_ = this->declare_parameter("publish_novatel_xyz_positions", publish_novatel_xyz_positions_);
    publish_novatel_utm_positions_ = this->declare_parameter("publish_novatel_utm_positions", publish_novatel_utm_positions_);
    publish_novatel_velocity_ = this->declare_parameter("publish_novatel_velocity", publish_novatel_velocity_);
    publish_novatel_heading2_ = this->declare_parameter("publish_novatel_heading2", publish_novatel_heading2_);
    publish_novatel_dual_antenna_heading_ = this->declare_parameter("publish_novatel_dual_antenna_heading", publish_novatel_dual_antenna_heading_);
    publish_novatel_psrdop2_ = this->declare_parameter("publish_novatel_psrdop2", publish_novatel_psrdop2_);
    publish_nmea_messages_ = this->declare_parameter("publish_nmea_messages", publish_nmea_messages_);
    publish_range_messages_ = this->declare_parameter("publish_range_messages", publish_range_messages_);
    publish_time_messages_ = this->declare_parameter("publish_time_messages", publish_time_messages_);
    publish_trackstat_ = this->declare_parameter("publish_trackstat", publish_trackstat_);
    publish_diagnostics_ = this->declare_parameter("publish_diagnostics", publish_diagnostics_);
    publish_sync_diagnostic_ = this->declare_parameter("publish_sync_diagnostic", publish_sync_diagnostic_);
    polling_period_ = this->declare_parameter("polling_period", polling_period_);
    reconnect_delay_s_ = this->declare_parameter("reconnect_delay_s", reconnect_delay_s_);
    use_binary_messages_ = this->declare_parameter("use_binary_messages", use_binary_messages_);
    span_frame_to_ros_frame_ = this->declare_parameter("span_frame_to_ros_frame", span_frame_to_ros_frame_);

    connection_type_ = this->declare_parameter("connection_type", connection_type_);
    connection_ = NovatelGps::ParseConnection(connection_type_);
    serial_baud_ = this->declare_parameter("serial_baud", serial_baud_);

    imu_frame_id_ = this->declare_parameter("imu_frame_id", std::string(""));
    frame_id_ = this->declare_parameter("frame_id", std::string(""));

    //set NovatelGps parameters
    gps_.gpsfix_sync_tol_ = this->declare_parameter("gpsfix_sync_tol", 0.01);
    gps_.wait_for_sync_ = this->declare_parameter("wait_for_sync", true);

    // Reset Service
    reset_service_ = this->create_service<novatel_gps_msgs::srv::NovatelFRESET>("freset",
                                                                                std::bind(
                                                                                    &NovatelGpsNode::resetService,
                                                                                    this, std::placeholders::_1,
                                                                                    std::placeholders::_2,
                                                                                    std::placeholders::_3));


    sync_sub_ = swri::Subscriber(*this, "gps_sync", 100,
                                 &NovatelGpsNode::SyncCallback, this);

    gps_pub_ = swri::advertise<gps_msgs::msg::GPSFix>(*this, "gps", 100);
    fix_pub_ = swri::advertise<sensor_msgs::msg::NavSatFix>(*this, "fix", 100);

    if (publish_clock_steering_)
    {
      clocksteering_pub_ = swri::advertise<novatel_gps_msgs::msg::ClockSteering>(*this, "clocksteering", 100);
    }

    if (publish_nmea_messages_)
    {
      gpgga_pub_ = swri::advertise<novatel_gps_msgs::msg::Gpgga>(*this, "gpgga", 100);
      gprmc_pub_ = swri::advertise<novatel_gps_msgs::msg::Gprmc>(*this, "gprmc", 100);
    }

    if (publish_gpgsa_)
    {
      gpgsa_pub_ = swri::advertise<novatel_gps_msgs::msg::Gpgsa>(*this, "gpgsa", 100);
    }

    if (publish_imu_messages_)
    {
      imu_pub_ = swri::advertise<sensor_msgs::msg::Imu>(*this, "imu", 100);
      novatel_imu_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelCorrectedImuData>(*this, "corrimudata", 100);
      insstdev_pub_ = swri::advertise<novatel_gps_msgs::msg::Insstdev>(*this, "insstdev", 100);
      inspva_pub_ = swri::advertise<novatel_gps_msgs::msg::Inspva>(*this, "inspva", 100);
      inspvax_pub_ = swri::advertise<novatel_gps_msgs::msg::Inspvax>(*this, "inspvax", 100);
      inscov_pub_ = swri::advertise<novatel_gps_msgs::msg::Inscov>(*this, "inscov", 100);
    }

    if (publish_gpgsv_)
    {
      gpgsv_pub_ = swri::advertise<novatel_gps_msgs::msg::Gpgsv>(*this, "gpgsv", 100);
    }

    if (publish_gphdt_)
    {
      gphdt_pub_ = swri::advertise<novatel_gps_msgs::msg::Gphdt>(*this, "gphdt", 100);
    }

    if (publish_novatel_positions_)
    {
      novatel_position_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelPosition>(*this, "bestpos", 100);
    }

    if (publish_novatel_xyz_positions_)
    {
      novatel_xyz_position_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelXYZ>(*this, "bestxyz", 100);
    }

    if (publish_novatel_utm_positions_)
    {
      novatel_utm_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelUtmPosition>(*this, "bestutm", 100);
    }

    if (publish_novatel_velocity_)
    {
      novatel_velocity_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelVelocity>(*this, "bestvel", 100);
    }
    else
    {
      gps_.wait_for_sync_ = false;
    }

    if (publish_novatel_heading2_)
    {
      novatel_heading2_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelHeading2>(*this, "heading2", 100);
    }

    if (publish_novatel_dual_antenna_heading_)
    {
      novatel_dual_antenna_heading_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelDualAntennaHeading>(*this,
                                                                                                            "dual_antenna_heading",
                                                                                                            100);
    }

    if (publish_novatel_psrdop2_)
    {
      novatel_psrdop2_pub_ = swri::advertise<novatel_gps_msgs::msg::NovatelPsrdop2>(*this,
          "psrdop2",
          100,
          true);
    }

    if (publish_range_messages_)
    {
      range_pub_ = swri::advertise<novatel_gps_msgs::msg::Range>(*this, "range", 100);
    }

    if (publish_time_messages_)
    {
      time_pub_ = swri::advertise<novatel_gps_msgs::msg::Time>(*this, "time", 100);
    }

    if (publish_trackstat_)
    {
      trackstat_pub_ = swri::advertise<novatel_gps_msgs::msg::Trackstat>(*this, "trackstat", 100);
    }

    hw_id_ = "Novatel GPS (" + device_ + ")";
    if (publish_diagnostics_)
    {
      diagnostic_updater_.setHardwareID(hw_id_);
      diagnostic_updater_.add("Connection",
                              this,
                              &NovatelGpsNode::DeviceDiagnostic);
      diagnostic_updater_.add("Hardware",
                              this,
                              &NovatelGpsNode::GpsDiagnostic);
      diagnostic_updater_.add("Data",
                              this,
                              &NovatelGpsNode::DataDiagnostic);
      diagnostic_updater_.add("Rate",
                              this,
                              &NovatelGpsNode::RateDiagnostic);
      diagnostic_updater_.add("GPS Fix",
                              this,
                              &NovatelGpsNode::FixDiagnostic);
      if (publish_sync_diagnostic_)
      {
        diagnostic_updater_.add("Sync",
                                this,
                                &NovatelGpsNode::SyncDiagnostic);
      }
    }


    gps_.ApplyVehicleBodyRotation(span_frame_to_ros_frame_);

    thread_ = boost::thread(&NovatelGpsNode::Spin, this);
    RCLCPP_INFO(this->get_logger(), "%s initialized", hw_id_.c_str());
  }

  NovatelGpsNode::~NovatelGpsNode()
  {
    gps_.Disconnect();
  }

  void NovatelGpsNode::SyncCallback(const builtin_interfaces::msg::Time::ConstSharedPtr& sync)
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    sync_times_.push_back(rclcpp::Time(*sync, this->get_clock()->get_clock_type()));
  }

  /**
   * Main spin loop connects to device, then reads data from it and publishes
   * messages.
   */
  void NovatelGpsNode::Spin()
  {
    std::string format_suffix;
    if (use_binary_messages_)
    {
      // NovAtel logs in binary format always end with "b", while ones in
      // ASCII format always end in "a".
      format_suffix = "b";
    }
    else
    {
      format_suffix = "a";
    }

    NovatelMessageOpts opts;
    opts["gpgga"] = polling_period_;
    opts["bestpos" + format_suffix] = polling_period_;  // Best position
    opts["bestvel" + format_suffix] = polling_period_;  // Best velocity
    opts["time" + format_suffix] = 1.0;  // Time
    if (publish_nmea_messages_)
    {
      opts["gprmc"] = polling_period_;
    }
    if (publish_novatel_xyz_positions_)
    {
      opts["bestxyz" + format_suffix] = polling_period_;
    }
    if (publish_novatel_utm_positions_)
    {
      opts["bestutm" + format_suffix] = polling_period_;
    }
    if (publish_novatel_heading2_)
    {
      opts["heading2" + format_suffix] = polling_period_;
    }
    if (publish_novatel_dual_antenna_heading_)
    {
      opts["dualantennaheading" + format_suffix] = polling_period_;
    }
    if (publish_novatel_psrdop2_)
    {
      opts["psrdop2" + format_suffix] = -1.0;
    }
    if (publish_gpgsa_)
    {
      opts["gpgsa"] = polling_period_;
    }
    if (publish_gpgsv_)
    {
      opts["gpgsv"] = 1.0;
    }
    if (publish_gphdt_)
    {
      opts["gphdt"] = polling_period_;
    }
    if (publish_clock_steering_)
    {
      opts["clocksteering" + format_suffix] = 1.0;
    }
    if (publish_imu_messages_)
    {
      double period = 1.0 / imu_rate_;
      opts["corrimudata" + format_suffix] = period;
      opts["inscov" + format_suffix] = 1.0;
      opts["inspva" + format_suffix] = period;
      opts["inspvax" + format_suffix] = period;
      opts["insstdev" + format_suffix] = 1.0;
      if (!use_binary_messages_)
      {
        RCLCPP_WARN(this->get_logger(), "Using the ASCII message format with CORRIMUDATA logs is not recommended.  "
                                        "A serial link will not be able to keep up with the data rate.");
      }

      // Only configure the imu rate if we set the param
      if (imu_sample_rate_ > 0)
      {
        gps_.SetImuRate(imu_sample_rate_, true);
      }
    }
    if (publish_range_messages_)
    {
      opts["range" + format_suffix] = 1.0;  // Range. 1 msg/sec is max rate
    }
    if (publish_trackstat_)
    {
      opts["trackstat" + format_suffix] = 1.0;  // Trackstat
    }
    // Set the serial baud rate if needed
    if (connection_ == NovatelGps::SERIAL)
    {
      gps_.SetSerialBaud(serial_baud_);
    }
    rclcpp::WallRate rate(1000.0);
    while (rclcpp::ok())
    {
      if (gps_.Connect(device_, connection_, opts))
      {
        // Connected to device. Begin reading/processing data
        RCLCPP_INFO(this->get_logger(), "%s connected to device", hw_id_.c_str());
        while (gps_.IsConnected() && rclcpp::ok())
        {
          // Read data from the device and publish any received messages
          try
          {
            CheckDeviceForData();
          }
          catch (const std::runtime_error& e)
          {
            RCLCPP_ERROR(this->get_logger(), "Error when checking for data: %s", e.what());
          }

          // Poke the diagnostic updater. It will only fire diagnostics if
          // its internal timer (1 Hz) has elapsed. Otherwise, this is a
          // noop
          if (publish_diagnostics_)
          {
            //diagnostic_updater_.force_update();
          }

          rate.sleep();
        }  // While (gps_.IsConnected() && ros::ok()) (inner loop to process data from device)
      }
      else  // Could not connect to the device
      {
        // TODO pjr Throttle this log when possible
        RCLCPP_ERROR(this->get_logger(), "Error connecting to device <%s:%s>: %s",
                     connection_type_.c_str(),
                     device_.c_str(),
                     gps_.ErrorMsg().c_str());
        device_errors_++;
        error_msg_ = gps_.ErrorMsg();
      }

      if (rclcpp::ok())
      {
        // If ROS is still OK but we got disconnected, we're going to try
        // to reconnect, but wait just a bit so we don't spam the device.
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<long>(reconnect_delay_s_ * 1000.0)));
      }

      rate.sleep();

      if (connection_ == NovatelGps::PCAP)
      {
        // If we're playing a PCAP file, we only want to play it once.
        rclcpp::shutdown();
      }
    }  // While (ros::ok) (outer loop to reconnect to device)

    gps_.Disconnect();
    RCLCPP_INFO(this->get_logger(), "%s disconnected and shut down", hw_id_.c_str());
  }

  /**
   * @brief Service request to reset the gps through FRESET
   */
  bool NovatelGpsNode::resetService(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const novatel_gps_msgs::srv::NovatelFRESET::Request::SharedPtr req,
                                    const novatel_gps_msgs::srv::NovatelFRESET::Response::SharedPtr res)
  {
    if (!gps_.IsConnected())
    {
      res->success = false;
    }

    // Formulate the reset command and send it to the device
    std::string command = "FRESET ";
    command += req->target.length() ? "STANDARD" : req->target;
    command += "\r\n";
    gps_.Write(command);

    if (req->target.length() == 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "No FRESET target specified. Doing FRESET STANDARD. This may be undesired behavior.");
    }

    res->success = true;
    return true;
  }

  /**
   * @brief Reads data from the device and publishes any parsed messages.
   *
   * Note that when reading from the device, this will block until data
   * is available.
   */
  void NovatelGpsNode::CheckDeviceForData()
  {
    std::vector<gps_msgs::msg::GPSFix::UniquePtr> fix_msgs;
    std::vector<novatel_gps_driver::BestposParser::MessageType> position_msgs;
    std::vector<novatel_gps_driver::GpggaParser::MessageType> gpgga_msgs;

    // This call appears to block if the serial device is disconnected
    NovatelGps::ReadResult result = gps_.ProcessData();
    if (result == NovatelGps::READ_ERROR)
    {
      // TODO pjr Throttle this when possible
      RCLCPP_ERROR(this->get_logger(), "Error reading from device <%s:%s>: %s",
                   connection_type_.c_str(),
                   device_.c_str(),
                   gps_.ErrorMsg().c_str());
      device_errors_++;
    }
    else if (result == NovatelGps::READ_TIMEOUT)
    {
      device_timeouts_++;
    }
    else if (result == NovatelGps::READ_INTERRUPTED)
    {
      // If we are interrupted by a signal, ROS is probably
      // quitting, but we'll wait for ROS to tell us to quit.
      device_interrupts_++;
    }
    else if (result == NovatelGps::READ_PARSE_FAILED)
    {
      RCLCPP_ERROR(this->get_logger(), "Error reading from device <%s:%s>: %s",
                   connection_type_.c_str(),
                   device_.c_str(),
                   gps_.ErrorMsg().c_str());
      gps_parse_failures_++;
    }
    else if (result == NovatelGps::READ_INSUFFICIENT_DATA)
    {
      gps_insufficient_data_warnings_++;
    }

    // GPSFix messages are always published, and Gpgga and Position messages
    // are used for generating some diagnostics.  Other message types will
    // only be retrieved if we're configured to publish them.
    gps_.GetFixMessages(fix_msgs);
    gps_.GetGpggaMessages(gpgga_msgs);
    gps_.GetNovatelPositions(position_msgs);

    // Increment the measurement count by the number of messages we just
    // read
    measurement_count_ += position_msgs.size();

    // If there are new position messages, store the most recent
    if (!position_msgs.empty())
    {
      last_novatel_position_ = position_msgs.back();
    }

    // Find all the gppga messages that are within 20 ms of whole
    // numbered UTC seconds and push their stamps onto the msg_times
    // buffer
    for (const auto& msg : gpgga_msgs)
    {
      if (msg->utc_seconds != 0)
      {
        auto second = static_cast<int64_t>(swri_math_util::Round(msg->utc_seconds));
        double difference = std::fabs(msg->utc_seconds - second);

        if (difference < 0.02)
        {
          msg_times_.push_back(msg->header.stamp);
        }
      }
    }

    // If timesync messages are available, CalculateTimeSync will
    // update a stat accumulator of the offset of the TimeSync message
    // stamp from the GPS message stamp
    CalculateTimeSync();

    // If TimeSync messages are available, CalculateTimeSync keeps
    // an acculumator of their offset, which is used to
    // calculate a rolling mean of the offset to apply to all messages
    rclcpp::Duration sync_offset(0); // If no TimeSyncs, assume 0 offset
    try
    {
      if (last_sync_ != rclcpp::Time(this->get_clock()->get_clock_type()))
      {
        sync_offset = std::chrono::duration<double>(stats::rolling_mean(rolling_offset_));
      }
      RCLCPP_DEBUG(this->get_logger(), "GPS TimeSync offset is %f", sync_offset.seconds());
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error comparing times: %s; last_sync_: %d",
          e.what(), last_sync_.get_clock_type());
    }

    if (publish_nmea_messages_)
    {
      for (auto& msg : gpgga_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        gpgga_pub_->publish(std::move(msg));
      }

      std::vector<novatel_gps_driver::GprmcParser::MessageType> gprmc_msgs;
      gps_.GetGprmcMessages(gprmc_msgs);
      for (auto& msg : gprmc_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        gprmc_pub_->publish(std::move(msg));
      }
    }

    if (publish_gpgsa_)
    {
      std::vector<novatel_gps_driver::GpgsaParser::MessageType> gpgsa_msgs;
      gps_.GetGpgsaMessages(gpgsa_msgs);
      for (auto& msg : gpgsa_msgs)
      {
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id_;
        gpgsa_pub_->publish(std::move(msg));
      }
    }

    if (publish_gpgsv_)
    {
      std::vector<novatel_gps_driver::GpgsvParser::MessageType> gpgsv_msgs;
      gps_.GetGpgsvMessages(gpgsv_msgs);
      for (auto& msg : gpgsv_msgs)
      {
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id_;
        gpgsv_pub_->publish(std::move(msg));
      }
    }

    if (publish_gphdt_)
    {
      std::vector<novatel_gps_driver::GphdtParser::MessageType> gphdt_msgs;
      gps_.GetGphdtMessages(gphdt_msgs);
      for (auto& msg : gphdt_msgs)
      {
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id_;
        gphdt_pub_->publish(std::move(msg));
      }
    }

    if (publish_novatel_positions_)
    {
      for (auto& msg : position_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_position_pub_->publish(*msg);
      }
    }

    if (publish_novatel_xyz_positions_)
    {
      std::vector<novatel_gps_driver::BestxyzParser::MessageType> xyz_position_msgs;
      gps_.GetNovatelXYZPositions(xyz_position_msgs);
      for (auto& msg : xyz_position_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_xyz_position_pub_->publish(std::move(msg));
      }
    }

    if (publish_novatel_utm_positions_)
    {
      std::vector<novatel_gps_driver::BestutmParser::MessageType> utm_msgs;
      gps_.GetNovatelUtmPositions(utm_msgs);
      for (auto& msg : utm_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_utm_pub_->publish(std::move(msg));
      }
    }

    if (publish_novatel_heading2_)
    {
      std::vector<novatel_gps_driver::Heading2Parser::MessageType> heading2_msgs;
      gps_.GetNovatelHeading2Messages(heading2_msgs);
      for (auto& msg : heading2_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_heading2_pub_->publish(std::move(msg));
      }
    }

    if (publish_novatel_dual_antenna_heading_)
    {
      std::vector<novatel_gps_driver::DualAntennaHeadingParser::MessageType> dual_antenna_heading_msgs;
      gps_.GetNovatelDualAntennaHeadingMessages(dual_antenna_heading_msgs);
      for (auto& msg : dual_antenna_heading_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_dual_antenna_heading_pub_->publish(std::move(msg));
      }
    }

    if (publish_novatel_psrdop2_)
    {
      std::vector<novatel_gps_driver::Psrdop2Parser::MessageType> psrdop2_msgs;
      gps_.GetNovatelPsrdop2Messages(psrdop2_msgs);
      for (auto& msg : psrdop2_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_psrdop2_pub_->publish(*msg);
      }
    }

    if (publish_clock_steering_)
    {
      std::vector<novatel_gps_driver::ClockSteeringParser::MessageType> msgs;
      gps_.GetClockSteeringMessages(msgs);
      for (auto& msg : msgs)
      {
        clocksteering_pub_->publish(std::move(msg));
      }
    }
    if (publish_novatel_velocity_)
    {
      std::vector<novatel_gps_driver::BestvelParser::MessageType> velocity_msgs;
      gps_.GetNovatelVelocities(velocity_msgs);
      for (auto& msg : velocity_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        novatel_velocity_pub_->publish(*msg);
      }
    }
    if (publish_time_messages_)
    {
      std::vector<novatel_gps_driver::TimeParser::MessageType> time_msgs;
      gps_.GetTimeMessages(time_msgs);
      for (auto& msg : time_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        time_pub_->publish(std::move(msg));
      }
    }
    if (publish_range_messages_)
    {
      std::vector<novatel_gps_driver::RangeParser::MessageType> range_msgs;
      gps_.GetRangeMessages(range_msgs);
      for (auto& msg : range_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        range_pub_->publish(std::move(msg));
      }
    }
    if (publish_trackstat_)
    {
      std::vector<novatel_gps_driver::TrackstatParser::MessageType> trackstat_msgs;
      gps_.GetTrackstatMessages(trackstat_msgs);
      for (auto& msg : trackstat_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = frame_id_;
        trackstat_pub_->publish(std::move(msg));
      }
    }
    if (publish_imu_messages_)
    {
      std::vector<novatel_gps_driver::CorrImuDataParser::MessageType> novatel_imu_msgs;
      gps_.GetNovatelCorrectedImuData(novatel_imu_msgs);
      for (auto& msg : novatel_imu_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = imu_frame_id_;
        novatel_imu_pub_->publish(*msg);
      }

      std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
      gps_.GetImuMessages(imu_msgs);
      for (const auto& msg : imu_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = imu_frame_id_;
        imu_pub_->publish(*msg);
      }

      std::vector<novatel_gps_driver::InscovParser::MessageType> inscov_msgs;
      gps_.GetInscovMessages(inscov_msgs);
      for (const auto& msg : inscov_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = imu_frame_id_;
        inscov_pub_->publish(*msg);
      }

      std::vector<novatel_gps_driver::InspvaParser::MessageType> inspva_msgs;
      gps_.GetInspvaMessages(inspva_msgs);
      for (auto& msg : inspva_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = imu_frame_id_;
        inspva_pub_->publish(*msg);
      }

      std::vector<novatel_gps_driver::InspvaxParser::MessageType> inspvax_msgs;
      gps_.GetInspvaxMessages(inspvax_msgs);
      for (auto& msg : inspvax_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = imu_frame_id_;
        inspvax_pub_->publish(std::move(msg));
      }

      std::vector<novatel_gps_driver::InsstdevParser::MessageType> insstdev_msgs;
      gps_.GetInsstdevMessages(insstdev_msgs);
      for (const auto& msg : insstdev_msgs)
      {
        msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
        msg->header.frame_id = imu_frame_id_;
        insstdev_pub_->publish(*msg);
      }
    }

    for (auto& msg : fix_msgs)
    {
      msg->header.stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()) + sync_offset;
      msg->header.frame_id = frame_id_;

      if (fix_pub_->get_subscription_count() > 0)
      {
        sensor_msgs::msg::NavSatFix::UniquePtr fix_msg = ConvertGpsFixToNavSatFix(msg);

        fix_pub_->publish(std::move(fix_msg));
      }

      // If the time between GPS message stamps is greater than 1.5
      // times the expected publish rate, increment the
      // publish_rate_warnings_ counter, which is used in diagnostics
      rclcpp::Time msgTime = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

      // Publish it; note that after this point, we no longer own the object
      if (publish_invalid_gpsfix_ || msg->status.status != gps_msgs::msg::GPSStatus::STATUS_NO_FIX)
      {
        gps_pub_->publish(std::move(msg));
      }

      try
      {
        if (last_published_.seconds() > 0 &&
            (msgTime - last_published_).seconds() > 1.5 * (1.0 / expected_rate_))
        {
          publish_rate_warnings_++;
        }
      }
      catch (const std::runtime_error& e)
      {
        RCLCPP_WARN(this->get_logger(), "last_published_ clock type (%d) wasn't the same as node's clock (%d)",
            last_published_.get_clock_type(), msgTime.get_clock_type());
      }

      last_published_ = msgTime;
    }
  }

  sensor_msgs::msg::NavSatFix::UniquePtr
  NovatelGpsNode::ConvertGpsFixToNavSatFix(const gps_msgs::msg::GPSFix::UniquePtr& msg)
  {
    sensor_msgs::msg::NavSatFix::UniquePtr fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    fix_msg->header = msg->header;
    fix_msg->latitude = msg->latitude;
    fix_msg->longitude = msg->longitude;
    fix_msg->altitude = msg->altitude;
    fix_msg->position_covariance = msg->position_covariance;
    switch (msg->status.status)
    {
      case gps_msgs::msg::GPSStatus::STATUS_NO_FIX:
        fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case gps_msgs::msg::GPSStatus::STATUS_FIX:
        fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
      case gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX:
        fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        break;
      case gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX:
        fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        break;
      case gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX:
      case gps_msgs::msg::GPSStatus::STATUS_WAAS_FIX:
      default:
        RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported fix status: %d", msg->status.status);
        fix_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }
    switch (msg->position_covariance_type)
    {
      case gps_msgs::msg::GPSFix::COVARIANCE_TYPE_KNOWN:
        fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
        break;
      case gps_msgs::msg::GPSFix::COVARIANCE_TYPE_APPROXIMATED:
        fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        break;
      case gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
        fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        break;
      case gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN:
        fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        break;
      default:
        RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported covariance type: %d", msg->position_covariance_type);
        fix_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        break;
    }
    // TODO Figure out how to set this based on info in a GPSFix
    fix_msg->status.service = 0;

    return fix_msg;
  }

  /**
   * Updates the time sync offsets by matching up timesync messages to gps
   * messages and calculating the time offset between them.
   */
  void NovatelGpsNode::CalculateTimeSync()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    int32_t synced_i = -1;  /// Index of last synced timesync msg
    int32_t synced_j = -1;  /// Index of last synced gps msg
    // Loop over sync times buffer
    for (size_t i = 0; i < sync_times_.size(); i++)
    {
      // Loop over message times buffer
      for (size_t j = synced_j + 1; j < msg_times_.size(); j++)
      {
        // Offset is the difference between the sync time and message time
        double offset = (sync_times_[i] - msg_times_[j]).seconds();
        if (std::fabs(offset) < 0.49)
        {
          // If the offset is less than 0.49 sec, the messages match
          synced_i = static_cast<int32_t>(i);
          synced_j = static_cast<int32_t>(j);
          // Add the offset to the stats accumulators
          offset_stats_(offset);
          rolling_offset_(offset);
          // Update the last sync
          last_sync_ = sync_times_[i];
          // Break out of the inner loop and continue looping through sync times
          break;
        }
      }
    }

    // Remove all the timesync messages that have been matched from the queue
    for (int i = 0; i <= synced_i && !sync_times_.empty(); i++)
    {
      sync_times_.pop_front();
    }

    // Remove all the gps messages that have been matched from the queue
    for (int j = 0; j <= synced_j && !msg_times_.empty(); j++)
    {
      msg_times_.pop_front();
    }
  }

  void NovatelGpsNode::FixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.clear();
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");

    if (!last_novatel_position_)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No Status");
      RCLCPP_WARN(this->get_logger(), "No GPS status data.");
      return;
    }

    status.add("Solution Status", last_novatel_position_->solution_status);
    status.add("Position Type", last_novatel_position_->position_type);
    status.add("Solution Age", last_novatel_position_->solution_age);
    status.add("Satellites Tracked", static_cast<int>(last_novatel_position_->num_satellites_tracked));
    status.add("Satellites Used", static_cast<int>(last_novatel_position_->num_satellites_used_in_solution));
    status.add("Software Version", last_novatel_position_->novatel_msg_header.receiver_software_version);

    const novatel_gps_msgs::msg::NovatelReceiverStatus& rcvr_status = last_novatel_position_->novatel_msg_header.receiver_status;
    status.add("Status Code", rcvr_status.original_status_code);

    if (last_novatel_position_->novatel_msg_header.receiver_status.original_status_code != 0)
    {
      uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      std::string msg = "Status Warning";
      // If the antenna is disconnected/broken, this is an error
      if (rcvr_status.antenna_is_open ||
          rcvr_status.antenna_is_shorted ||
          !rcvr_status.antenna_powered)
      {
        msg += " Antenna Problem";
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      }
      status.add("Error Flag", rcvr_status.error_flag ? "true" : "false");
      status.add("Temperature Flag", rcvr_status.temperature_flag ? "true" : "false");
      status.add("Voltage Flag", rcvr_status.voltage_supply_flag ? "true" : "false");
      status.add("Antenna Not Powered", rcvr_status.antenna_powered ? "false" : "true");
      status.add("Antenna Open", rcvr_status.antenna_is_open ? "true" : "false");
      status.add("Antenna Shorted", rcvr_status.antenna_is_shorted ? "true" : "false");
      status.add("CPU Overloaded", rcvr_status.cpu_overload_flag ? "true" : "false");
      status.add("COM1 Buffer Overrun", rcvr_status.com1_buffer_overrun ? "true" : "false");
      status.add("COM2 Buffer Overrun", rcvr_status.com2_buffer_overrun ? "true" : "false");
      status.add("COM3 Buffer Overrun", rcvr_status.com3_buffer_overrun ? "true" : "false");
      status.add("USB Buffer Overrun", rcvr_status.usb_buffer_overrun ? "true" : "false");
      status.add("RF1 AGC Flag", rcvr_status.rf1_agc_flag ? "true" : "false");
      status.add("RF2 AGC Flag", rcvr_status.rf2_agc_flag ? "true" : "false");
      status.add("Almanac Flag", rcvr_status.almanac_flag ? "true" : "false");
      status.add("Position Solution Flag", rcvr_status.position_solution_flag ? "true" : "false");
      status.add("Position Fixed Flag", rcvr_status.position_fixed_flag ? "true" : "false");
      status.add("Clock Steering Status", rcvr_status.clock_steering_status_enabled ? "true" : "false");
      status.add("Clock Model Flag", rcvr_status.clock_model_flag ? "true" : "false");
      status.add("OEMV External Oscillator Flag", rcvr_status.oemv_external_oscillator_flag ? "true" : "false");
      status.add("Software Resource Flag", rcvr_status.software_resource_flag ? "true" : "false");
      status.add("Auxiliary1 Flag", rcvr_status.aux1_status_event_flag ? "true" : "false");
      status.add("Auxiliary2 Flag", rcvr_status.aux2_status_event_flag ? "true" : "false");
      status.add("Auxiliary3 Flag", rcvr_status.aux3_status_event_flag ? "true" : "false");
      RCLCPP_WARN(this->get_logger(), "Novatel status code: %d", rcvr_status.original_status_code);
      status.summary(level, msg);
    }
  }

  void NovatelGpsNode::SyncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");

    if (last_sync_ == rclcpp::Time(this->get_clock()->get_clock_type()))
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No Sync");
      return;
    }
    else if (last_sync_ < this->get_clock()->now() - std::chrono::seconds(10))
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Sync Stale");
      RCLCPP_ERROR(this->get_logger(), "GPS time synchronization is stale.");
    }

    status.add("Last Sync", last_sync_.seconds());
    status.add("Mean Offset", stats::mean(offset_stats_));
    status.add("Mean Offset (rolling)", stats::rolling_mean(rolling_offset_));
    status.add("Offset Variance", stats::variance(offset_stats_));
    status.add("Min Offset", stats::min(offset_stats_));
    status.add("Max Offset", stats::max(offset_stats_));
  }

  void NovatelGpsNode::DeviceDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");

    if (device_errors_ > 0)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device Errors");
    }
    else if (device_interrupts_ > 0)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Device Interrupts");
      RCLCPP_WARN(this->get_logger(), "device interrupts detected <%s:%s>: %d",
                  connection_type_.c_str(), device_.c_str(), device_interrupts_);
    }
    else if (device_timeouts_)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Device Timeouts");
      RCLCPP_WARN(this->get_logger(), "device timeouts detected <%s:%s>: %d",
                  connection_type_.c_str(), device_.c_str(), device_timeouts_);
    }

    status.add("Errors", device_errors_);
    status.add("Interrupts", device_interrupts_);
    status.add("Timeouts", device_timeouts_);

    device_timeouts_ = 0;
    device_interrupts_ = 0;
    device_errors_ = 0;
  }

  void NovatelGpsNode::GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");

    if (gps_parse_failures_ > 0)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Parse Failures");
      RCLCPP_WARN(this->get_logger(), "gps parse failures detected <%s>: %d",
                  hw_id_.c_str(), gps_parse_failures_);
    }

    status.add("Parse Failures", gps_parse_failures_);
    status.add("Insufficient Data Warnings", gps_insufficient_data_warnings_);

    gps_parse_failures_ = 0;
    gps_insufficient_data_warnings_ = 0;
  }

  void NovatelGpsNode::DataDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {

    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");

    double period = diagnostic_updater_.getPeriod().seconds();
    double measured_rate = measurement_count_ / period;

    if (measured_rate < 0.5 * expected_rate_)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Insufficient Data Rate");
      RCLCPP_ERROR(this->get_logger(), "insufficient data rate <%s>: %lf < %lf",
                   hw_id_.c_str(), measured_rate, expected_rate_);
    }
    else if (measured_rate < 0.95 * expected_rate_)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Insufficient Data Rate");
      RCLCPP_WARN(this->get_logger(), "insufficient data rate <%s>: %lf < %lf",
                  hw_id_.c_str(), measured_rate, expected_rate_);
    }

    status.add("Measurement Rate (Hz)", measured_rate);

    measurement_count_ = 0;
  }

  void NovatelGpsNode::RateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal Publish Rate");

    bool gap_detected = false;
    try
    {
      double elapsed = (this->get_clock()->now() - last_published_).seconds();
      if (elapsed > 2.0 / expected_rate_)
      {
        publish_rate_warnings_++;
        gap_detected = true;
      }
    }
    catch (std::runtime_error& e)
    {
      // If we have an exception comparing clocks, that means last_published_
      // has never been updated
      gap_detected = true;
    }

    if (publish_rate_warnings_ > 1 || gap_detected)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Insufficient Publish Rate");
      RCLCPP_WARN(this->get_logger(), "publish rate failures detected <%s>: %d",
                  hw_id_.c_str(), publish_rate_warnings_);
    }

    status.add("Warnings", publish_rate_warnings_);

    publish_rate_warnings_ = 0;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(novatel_gps_driver::NovatelGpsNode)
