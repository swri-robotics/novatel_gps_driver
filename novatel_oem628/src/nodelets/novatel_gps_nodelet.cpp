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

/**
 * \file
 *
 * This nodelet is a driver for Novatel OEM6 and FlexPack6 GPS receivers. It
 * publishes standard ROS gps_common/GPSFix messages, as well as custom
 * NovatelPosition, GPGGA, and GPRMC messages.
 *
 * Note: At this time, only serial connections to Novatel devices are
 * supported. Future versions of this driver may support TCP and UDP
 * connections.
 *
 * <b>Topics Subscribed:</b>
 *
 * \e gps_sync <tt>std_msgs/Time<tt> - Timestamped sync pulses
 *    from a DIO module (optional). These are used to improve the accuracy of
 *    the time stamps of the messages published.
 *
 * <b>Topics Published:</b>
 *
 * \e gps <tt>gps_common/GPSFix</tt> - GPS data for navigation
 * \e gppga <tt>novatel_oem628/Gpgga</tt> - Raw GPGGA data for debugging (only
 *    published if `publish_nmea_messages` is set `true`)
 * \e gpgsa <tt>novatel_gps_msgs/Gpgga</tt> - Raw GPGSA data for debugging (only
 *    published if `publish_gpgsa` is set `true`)
 * \e gprmc <tt>novatel_gps_msgs/Gprmc</tt> - Raw GPRMC data for debugging (only
 *    published if `publish_nmea_messages` is set `true`)
 * \e bestpos <tt>novatel_gps_msgs/NovatelPosition</tt> - High fidelity Novatel-
 *    specific position and receiver status data. (only published if
 *    `publish_novatel_positions` is set `true`)
 * \e bestvel <tt>novatel_gps_msgs/NovatelVelocity</tt> - High fidelity Novatel-
 *    specific velocity and receiver status data. (only published if
 *    `publish_novatel_velocity` is set `true`)
 * \e time <tt>novatel_gps_msgs/Time</tt> - Novatel-specific time data. (Only
 *    published if `publish_time` is set `true`.)
 * \e trackstat <tt>novatel_gps_msgs/Trackstat</tt> - Novatel-specific trackstat
 *    data at 1 Hz. (Only published if `publish_trackstat` is set `true`.)
 *
 * <b>Parameters:</b>
 *
 * \e buffer_capacity <tt>int</tt> - Size of the circular buffers used to sync
 *    GPGGA, GPRMC, and BESTPOS messages. [10]
 * \e connection_type <tt>str</tt> - "serial", "udp", or "tcp" as appropriate
 *    for the Novatel device connected. Only "serial" is supported at this
 *    time. ["serial"]
 * \e device <tt>str</tt> - The path to the serial device, e.g. /dev/ttyUSB0
 *    [""]
 * \e frame_id <tt>str</tt> - The TF frame ID to set in all published message
 *    headers. [""]
 * \e gpgga_gprmc_sync_tol <tt>dbl</tt> - Sync tolarance (seconds) for syncing
 *    GPGGA messages with GPRMC messages. [0.01]
 * \e gpgga_position_sync_tol <tt>dbl</tt> - Sync tolarance (seconds) for
 *    syncing GPGGA messages with BESTPOS messages. [0.01]
 * \e ignore_sync_diagnostic <tt>bool</tt> - If true, publish a Sync diagnostic
 *    [false]
 * \e polling_period <tt>dbl</tt> - The number of seconds in between messages
 *    requested from the GPS. (Does not affect time messages) [0.05]
 * \e publish_diagnostics <tt>bool</tt> - If set true, the driver publishes
 *    ROS diagnostics [true]
 * \e publish_gpgsa <tt>bool</tt> - If set true, the driver requests GPGSA
 *    messages from the device at 20 Hz and publishes them on `gpgsa`
 * \e publish_nmea_messages <tt>bool</tt> - If set true, the driver publishes
 *    GPGGA and GPRMC messages (see Topics Published) [false]
 * \e publish_novatel_messages <tt>bool</tt> - If set true, the driver
 *    publishes Novatel Bestpos messages (see Topics Published) [false]
 * \e publish_novatel_velocity <tt>bool</tt> - If set true, the driver
 *    publishes Novatel Bestvel messages (see Topics Published) [false]
 * \e publish_time <tt>bool</tt> - If set true, the driver publishes Novatel
 *    Time messages (see Topics Published) [false]
 * \e publish_trackstat <tt>bool</tt> - If set true, the driver publishes
 *    Novatel Trackstat messages (see Topics Published) [false]
 * \e wait_for_position <tt>bool</tt> - ??? [false]
 */
#include <exception>
#include <string>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/tail.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/circular_buffer.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <gps_common/GPSFix.h>
#include <nodelet/nodelet.h>
#include <novatel_gps_msgs/NovatelMessageHeader.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <novatel_gps_msgs/Gpgga.h>
#include <novatel_gps_msgs/Gprmc.h>
#include <novatel_gps_msgs/NovatelMessageHeader.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/Range.h>
#include <novatel_gps_msgs/Time.h>
#include <novatel_oem628/novatel_gps.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <swri_math_util/math_util.h>
#include <swri_roscpp/parameters.h>
#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>

namespace stats = boost::accumulators;

namespace novatel_oem628
{
  class NovatelGpsNodelet : public nodelet::Nodelet
  {
  public:
    NovatelGpsNodelet() :
      device_(""),
      connection_type_("serial"),
      polling_period_(0.05),
      publish_gpgsa_(false),
      publish_gpgsv_(false),
      publish_novatel_positions_(false),
      publish_novatel_velocity_(false),
      publish_nmea_messages_(false),
      publish_range_messages_(false),
      publish_time_messages_(false),
      publish_trackstat_(false),
      publish_diagnostics_(true),
      ignore_sync_diagnostic(false),
      connection_(NovatelGps::SERIAL),
      last_sync_(ros::TIME_MIN),
      rolling_offset_(stats::tag::rolling_window::window_size = 10),
      expected_rate_(20),
      device_timeouts_(0),
      device_interrupts_(0),
      device_errors_(0),
      gps_parse_failures_(0),
      gps_insufficient_data_warnings_(0),
      publish_rate_warnings_(0),
      measurement_count_(0),
      last_published_(ros::TIME_MIN),
      frame_id_("")
    {
    }

    ~NovatelGpsNodelet()
    {
      gps_.Disconnect();
    }

    /**
     * Init method reads parameters and sets up publishers and subscribers.
     * It does not connect to the device.
     */
    void onInit()
    {
      ros::NodeHandle &node = getNodeHandle();
      ros::NodeHandle &priv = getPrivateNodeHandle();

      swri::param(priv,"device", device_, device_);
      swri::param(priv,"publish_gpgsa", publish_gpgsa_, publish_gpgsa_);
      swri::param(priv,"publish_gpgsv", publish_gpgsv_, publish_gpgsv_);
      swri::param(priv,"publish_novatel_positions", publish_novatel_positions_, publish_novatel_positions_);
      swri::param(priv,"publish_novatel_velocity", publish_novatel_velocity_, publish_novatel_velocity_);
      swri::param(priv,"publish_nmea_messages", publish_nmea_messages_, publish_nmea_messages_);
      swri::param(priv,"publish_range_messages", publish_range_messages_, publish_range_messages_);
      swri::param(priv,"publish_time_messages", publish_time_messages_, publish_time_messages_);
      swri::param(priv,"publish_trackstat", publish_trackstat_, publish_trackstat_);
      swri::param(priv,"publish_diagnostics", publish_diagnostics_, publish_diagnostics_);
      swri::param(priv,"ignore_sync_diagnostic", ignore_sync_diagnostic, ignore_sync_diagnostic);
      swri::param(priv, "polling_period", polling_period_, polling_period_);

      swri::param(priv,"connection_type", connection_type_, connection_type_);
      connection_ = NovatelGps::ParseConnection(connection_type_);

      swri::param(priv,"frame_id", frame_id_, std::string(""));
      
      //set NovatelGps parameters
      swri::param(priv,"gpgga_gprmc_sync_tol", gps_.gpgga_gprmc_sync_tol, 0.01);
      swri::param(priv,"gpgga_position_sync_tol", gps_.gpgga_position_sync_tol, 0.01);
      swri::param(priv,"wait_for_position", gps_.wait_for_position, false);
      int buffer_capacity;
      swri::param(priv,"buffer_capacity", buffer_capacity, 10);
      gps_.setBufferCapacity(buffer_capacity);

      sync_sub_ = swri::Subscriber(node,"gps_sync", 100, &NovatelGpsNodelet::SyncCallback, this);

      std::string gps_topic = node.resolveName("gps");
      gps_pub_ = swri::advertise<gps_common::GPSFix>(node,gps_topic, 100);

      if (publish_nmea_messages_)
      {
        gpgga_pub_ = swri::advertise<novatel_gps_msgs::Gpgga>(node,"gpgga", 100);
        gprmc_pub_ = swri::advertise<novatel_gps_msgs::Gprmc>(node,"gprmc", 100);
      }

      if (publish_gpgsa_)
      {
        gpgsa_pub_ = swri::advertise<novatel_gps_msgs::Gpgsa>(node, "gpgsa", 100);
      }

      if (publish_gpgsv_)
      {
        gpgsv_pub_ = swri::advertise<novatel_gps_msgs::Gpgsv>(node, "gpgsv", 100);
      }

      if (publish_novatel_positions_)
      { 
        novatel_position_pub_ = swri::advertise<novatel_gps_msgs::NovatelPosition>(node, "bestpos", 100);
      }

      if (publish_novatel_velocity_)
      {
        novatel_velocity_pub_ = swri::advertise<novatel_gps_msgs::NovatelVelocity>(node, "bestvel", 100);
      }

      if (publish_range_messages_)
      {
        range_pub_ = swri::advertise<novatel_gps_msgs::Range>(node, "range", 100);
      }

      if (publish_time_messages_)
      {
        time_pub_ = swri::advertise<novatel_gps_msgs::Time>(node, "time", 100);
      }

      if (publish_trackstat_)
      {
        trackstat_pub_ = swri::advertise<novatel_gps_msgs::Trackstat>(node, "trackstat", 100);
      }

      hw_id_ = "Novatel GPS (" + device_ +")";
      if (publish_diagnostics_)
      {
        diagnostic_updater_.setHardwareID(hw_id_);
        diagnostic_updater_.add("Connection",
            this,
            &NovatelGpsNodelet::DeviceDiagnostic);
        diagnostic_updater_.add("Hardware",
            this,
            &NovatelGpsNodelet::GpsDiagnostic);
        diagnostic_updater_.add("Data",
            this,
            &NovatelGpsNodelet::DataDiagnostic);
        diagnostic_updater_.add("Rate",
            this,
            &NovatelGpsNodelet::RateDiagnostic);
        diagnostic_updater_.add("GPS Fix",
            this,
            &NovatelGpsNodelet::FixDiagnostic);
        if (!ignore_sync_diagnostic)
        {
          diagnostic_updater_.add("Sync",
              this,
              &NovatelGpsNodelet::SyncDiagnostic);
        }
      }
      thread_ = boost::thread(&NovatelGpsNodelet::Spin, this);
      NODELET_INFO("%s initialized", hw_id_.c_str());
    }

    void SyncCallback(const std_msgs::TimeConstPtr& sync)
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      sync_times_.push_back(sync->data);
    }

    /**
     * Main spin loop connects to device, then reads data from it and publishes
     * messages.
     */
    void Spin()
    {
      std::vector<novatel_gps_msgs::NovatelPositionPtr> position_msgs;
      std::vector<gps_common::GPSFixPtr> fix_msgs;
      std::vector<novatel_gps_msgs::GpggaPtr> gpgga_msgs;
      std::vector<novatel_gps_msgs::GprmcPtr> gprmc_msgs;

      NovatelMessageOpts opts;
      opts["gpgga"] = polling_period_;
      opts["gprmc"] = polling_period_;
      opts["bestposa"] = polling_period_;  // Best position (ASCII)
      opts["timea"] = 1.0;  // Time (ASCII)

      if (publish_gpgsa_)
      {
        opts["gpgsa"] = polling_period_;
      }
      if (publish_gpgsv_)
      {
        opts["gpgsv"] = polling_period_;
      }
      if (publish_novatel_velocity_)
      {
        opts["bestvela"] = polling_period_;  // Best velocity (ASCII)
      }
      if (publish_range_messages_)
      {
         opts["rangea"] = 1.0;  // Range (ASCII). 1 msg/sec is max rate
      }
      if (publish_trackstat_)
      {
         opts["trackstata"] = 1.0;  // Trackstat (ASCII)
      }
      while (ros::ok())
      {
        if (gps_.Connect(device_, connection_, opts))
        {
          // Connected to device. Begin reading/processing data
          NODELET_INFO("%s connected to device", hw_id_.c_str());
          while (ros::ok())
          {
            // This call appears to block if the serial device is disconnected
            NovatelGps::ReadResult result = gps_.ProcessData();
            if (result == NovatelGps::READ_ERROR)
            {
              NODELET_ERROR_THROTTLE(1, "Error reading from device <%s:%s>: %s",
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
              gps_parse_failures_++;
            }
            else if (result == NovatelGps::READ_INSUFFICIENT_DATA)
            {
              gps_insufficient_data_warnings_++;
            }

            // Read messages from the driver into the local message lists
            gps_.GetGpggaMessages(gpgga_msgs);
            gps_.GetGprmcMessages(gprmc_msgs);
            gps_.GetNovatelPositions(position_msgs);
            gps_.GetFixMessages(fix_msgs);

            // Increment the measurement count by the number of messages we just
            // read
            measurement_count_ += gpgga_msgs.size();

            // If there are new position messages, store the most recent
            if (!position_msgs.empty())
            {
              last_novatel_position_ = position_msgs.back();
            }

            // Find all the gppga messages that are within 20 ms of whole
            // numbered UTC seconds and push their stamps onto the msg_times
            // buffer
            for (size_t i = 0; i < gpgga_msgs.size(); i++)
            {
              if (gpgga_msgs[i]->utc_seconds != 0)
              {
                int64_t second = swri_math_util::Round(gpgga_msgs[i]->utc_seconds);
                double difference = std::fabs(gpgga_msgs[i]->utc_seconds - second);

                if (difference < 0.02)
                {
                  msg_times_.push_back(gpgga_msgs[i]->header.stamp);
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
            ros::Duration sync_offset(0); // If no TimeSyncs, assume 0 offset
            if (last_sync_ != ros::TIME_MIN)
            {
              sync_offset = ros::Duration(stats::rolling_mean(rolling_offset_));
            }
            ROS_DEBUG_STREAM("GPS TimeSync offset is " << sync_offset);

            if (publish_nmea_messages_)
            {
              for (size_t i = 0; i < gpgga_msgs.size(); i++)
              {
                gpgga_msgs[i]->header.stamp += sync_offset;
                gpgga_msgs[i]->header.frame_id = frame_id_;
                gpgga_pub_.publish(gpgga_msgs[i]);
              }

              for (size_t i = 0; i < gprmc_msgs.size(); i++)
              {
                gprmc_msgs[i]->header.stamp += sync_offset;
                gprmc_msgs[i]->header.frame_id = frame_id_;
                gprmc_pub_.publish(gprmc_msgs[i]);
              }
            }

            if (publish_gpgsa_)
            {
              std::vector<novatel_gps_msgs::GpgsaPtr> gpgsa_msgs;
              gps_.GetGpgsaMessages(gpgsa_msgs);
              for (size_t i = 0; i < gpgsa_msgs.size(); ++i)
              {
                gpgsa_msgs[i]->header.stamp = ros::Time::now();;
                gpgsa_msgs[i]->header.frame_id = frame_id_;
                gpgsa_pub_.publish(gpgsa_msgs[i]);
              }
            }

            if (publish_gpgsv_)
            {
              std::vector<novatel_gps_msgs::GpgsvPtr> gpgsv_msgs;
              gps_.GetGpgsvMessages(gpgsv_msgs);
              for (size_t i = 0; i < gpgsv_msgs.size(); ++i)
              {
                gpgsv_msgs[i]->header.stamp = ros::Time::now();;
                gpgsv_msgs[i]->header.frame_id = frame_id_;
                gpgsv_pub_.publish(gpgsv_msgs[i]);
              }
            }

            if (publish_novatel_positions_)
            {
              for (size_t i = 0; i < position_msgs.size(); i++)
              {
                position_msgs[i]->header.stamp += sync_offset;
                position_msgs[i]->header.frame_id = frame_id_;
                novatel_position_pub_.publish(position_msgs[i]);
              }
            }

            if (publish_novatel_velocity_)
            {
              std::vector<novatel_gps_msgs::NovatelVelocityPtr> velocity_msgs;
              gps_.GetNovatelVelocities(velocity_msgs);
              for (size_t i = 0; i < velocity_msgs.size(); i++)
              {
                velocity_msgs[i]->header.stamp += sync_offset;
                velocity_msgs[i]->header.frame_id = frame_id_;
                novatel_velocity_pub_.publish(velocity_msgs[i]);
              }
            }
            if (publish_time_messages_)
            {
              std::vector<novatel_gps_msgs::TimePtr> time_msgs;
              gps_.GetTimeMessages(time_msgs);
              for (size_t i = 0; i < time_msgs.size(); i++)
              {
                time_msgs[i]->header.stamp += sync_offset;
                time_msgs[i]->header.frame_id = frame_id_;
                time_pub_.publish(time_msgs[i]);
              }
            }
            if (publish_range_messages_)
            {
              std::vector<novatel_gps_msgs::RangePtr> range_msgs;
              gps_.GetRangeMessages(range_msgs);
              for (size_t i = 0; i < range_msgs.size(); i++)
              {
                range_msgs[i]->header.stamp += sync_offset;
                range_msgs[i]->header.frame_id = frame_id_;
                range_pub_.publish(range_msgs[i]);
              }
            }
            if (publish_trackstat_)
            {
              std::vector<novatel_gps_msgs::TrackstatPtr> trackstat_msgs;
              gps_.GetTrackstatMessages(trackstat_msgs);
              ROS_DEBUG("Got %zu trackstat msgs", trackstat_msgs.size());
              for (size_t i = 0; i < trackstat_msgs.size(); i++)
              {
                trackstat_msgs[i]->header.stamp += sync_offset;
                trackstat_msgs[i]->header.frame_id = frame_id_;
                trackstat_pub_.publish(trackstat_msgs[i]);
              }

            }

            for (size_t i = 0; i < fix_msgs.size(); i++)
            {
              fix_msgs[i]->header.stamp += sync_offset;
              fix_msgs[i]->header.frame_id = frame_id_;
              gps_pub_.publish(fix_msgs[i]);

              // If the time between GPS message stamps is greater than 1.5
              // times the expected publish rate, increment the
              // publish_rate_warnings_ counter, which is used in diagnostics
              if (last_published_ != ros::TIME_MIN &&
                  (fix_msgs[i]->header.stamp - last_published_).toSec() > 1.5 * (1.0 / expected_rate_))
              {
                publish_rate_warnings_++;
              }

              last_published_ = fix_msgs[i]->header.stamp;
            }

            // Poke the diagnostic updater. It will only fire diagnostics if
            // its internal timer (1 Hz) has elapsed. Otherwise, this is a
            // noop
            if (publish_diagnostics_)
            {
              diagnostic_updater_.update();
            }

            // Spin once to let the ROS callbacks fire
            ros::spinOnce();
            // Sleep for a microsecond to prevent CPU hogging
            boost::this_thread::sleep(boost::posix_time::microseconds(1));
          }  // While (ros::ok) (inner loop to process data from device)
        }
        else  // Could not connect to the device
        {
          NODELET_ERROR_THROTTLE(1, "Error connecting to device <%s:%s>: %s",
            connection_type_.c_str(),
            device_.c_str(),
            gps_.ErrorMsg().c_str());
          device_errors_++;
          error_msg_ = gps_.ErrorMsg();
        }

        // Poke the diagnostic updater. It will only fire diagnostics if
        // its internal timer (1 Hz) has elapsed. Otherwise, this is a
        // noop
        if (publish_diagnostics_)
        {
          diagnostic_updater_.update();
        }

        // Spin once to let the ROS callbacks fire
        ros::spinOnce();
        // Sleep for a microsecond to prevent CPU hogging
        boost::this_thread::sleep(boost::posix_time::microseconds(1));
      }  // While (ros::ok) (outer loop to reconnect to device)

      gps_.Disconnect();
      NODELET_INFO("%s disconnected and shut down", hw_id_.c_str());
    }

  private:
    // Parameters
    /// The device identifier e.g. /dev/ttyUSB0
    std::string device_;
    /// The connection type, ("serial", "tcp", or "udp")
    std::string connection_type_;
    double polling_period_;
    bool publish_gpgsa_;
    bool publish_gpgsv_;
    bool publish_novatel_positions_;
    bool publish_novatel_velocity_;
    bool publish_nmea_messages_;
    bool publish_range_messages_;
    bool publish_time_messages_;
    bool publish_trackstat_;
    bool publish_diagnostics_;
    bool ignore_sync_diagnostic;

    ros::Publisher gps_pub_;
    ros::Publisher novatel_position_pub_;
    ros::Publisher novatel_velocity_pub_;
    ros::Publisher gpgga_pub_;
    ros::Publisher gpgsv_pub_;
    ros::Publisher gpgsa_pub_;
    ros::Publisher gprmc_pub_;
    ros::Publisher range_pub_;
    ros::Publisher time_pub_;
    ros::Publisher trackstat_pub_;

    NovatelGps::ConnectionType connection_;
    NovatelGps gps_;

    boost::thread thread_;
    boost::mutex mutex_;

    /// Subscriber to listen for sync times from a DIO
    swri::Subscriber sync_sub_;
    ros::Time last_sync_;
    /// Buffer of sync message time stamps
    boost::circular_buffer<ros::Time> sync_times_;
    /// Buffer of gps message time stamps
    boost::circular_buffer<ros::Time> msg_times_;
    /// Stats on time offset
    stats::accumulator_set<float, stats::stats<
      stats::tag::max,
      stats::tag::min,
      stats::tag::mean,
      stats::tag::variance> > offset_stats_;
    /// Rolling mean of time offset
    stats::accumulator_set<float, stats::stats<stats::tag::rolling_mean> > rolling_offset_;

    // ROS diagnostics
    gps_common::GPSFix last_gps_;
    std::string error_msg_;
    diagnostic_updater::Updater diagnostic_updater_;
    std::string hw_id_;
    double expected_rate_;
    int32_t device_timeouts_;
    int32_t device_interrupts_;
    int32_t device_errors_;
    int32_t gps_parse_failures_;
    int32_t gps_insufficient_data_warnings_;
    int32_t publish_rate_warnings_;
    int32_t measurement_count_;
    ros::Time last_published_;
    novatel_gps_msgs::NovatelPositionPtr last_novatel_position_;

    std::string frame_id_;
    
    /**
     * Updates the time sync offsets by matching up timesync messages to gps
     * messages and calculating the time offset between them.
     */
    void CalculateTimeSync()
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
          double offset = (sync_times_[i] - msg_times_[j]).toSec();
          if (std::fabs(offset) < 0.49)
          {
            // If the offset is less than 0.49 sec, the messages match
            synced_i = i;
            synced_j = j;
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

    void FixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status)
    {
      status.clear();
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

      if (!last_novatel_position_)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No Status");
        NODELET_WARN("No GPS status data.");
        return;
      }

      status.add("Solution Status", last_novatel_position_->solution_status);
      status.add("Position Type", last_novatel_position_->position_type);
      status.add("Solution Age", last_novatel_position_->solution_age);
      status.add("Satellites Tracked", static_cast<int>(last_novatel_position_->num_satellites_tracked));
      status.add("Satellites Used", static_cast<int>(last_novatel_position_->num_satellites_used_in_solution));
      status.add("Software Version", last_novatel_position_->novatel_msg_header.receiver_software_version);

      const novatel_gps_msgs::NovatelReceiverStatus& rcvr_status = last_novatel_position_->novatel_msg_header.receiver_status;
      status.add("Status Code", rcvr_status.original_status_code);

      if (last_novatel_position_->novatel_msg_header.receiver_status.original_status_code != 0)
      {
        int8_t level = diagnostic_msgs::DiagnosticStatus::WARN;
        std::string msg = "Status Warning";
        // If the antenna is disconnected/broken, this is an error
        if (rcvr_status.antenna_is_open ||
            rcvr_status.antenna_is_shorted ||
            !rcvr_status.antenna_powered)
        {
          msg += " Antenna Problem";
          level = diagnostic_msgs::DiagnosticStatus::ERROR;
        }
        status.add("Error Flag", rcvr_status.error_flag?"true":"false");
        status.add("Temperature Flag", rcvr_status.temperature_flag?"true":"false");
        status.add("Voltage Flag", rcvr_status.voltage_supply_flag?"true":"false");
        status.add("Antenna Not Powered", rcvr_status.antenna_powered?"false":"true");
        status.add("Antenna Open", rcvr_status.antenna_is_open?"true":"false");
        status.add("Antenna Shorted", rcvr_status.antenna_is_shorted?"true":"false");
        status.add("CPU Overloaded", rcvr_status.cpu_overload_flag?"true":"false");
        status.add("COM1 Buffer Overrun", rcvr_status.com1_buffer_overrun?"true":"false");
        status.add("COM2 Buffer Overrun", rcvr_status.com2_buffer_overrun?"true":"false");
        status.add("COM3 Buffer Overrun", rcvr_status.com3_buffer_overrun?"true":"false");
        status.add("USB Buffer Overrun", rcvr_status.usb_buffer_overrun?"true":"false");
        status.add("RF1 AGC Flag", rcvr_status.rf1_agc_flag?"true":"false");
        status.add("RF2 AGC Flag", rcvr_status.rf2_agc_flag?"true":"false");
        status.add("Almanac Flag", rcvr_status.almanac_flag?"true":"false");
        status.add("Position Solution Flag", rcvr_status.position_solution_flag?"true":"false");
        status.add("Position Fixed Flag", rcvr_status.position_fixed_flag?"true":"false");
        status.add("Clock Steering Status", rcvr_status.clock_steering_status_enabled?"true":"false");
        status.add("Clock Model Flag", rcvr_status.clock_model_flag?"true":"false");
        status.add("OEMV External Oscillator Flag", rcvr_status.oemv_external_oscillator_flag?"true":"false");
        status.add("Software Resource Flag", rcvr_status.software_resource_flag?"true":"false");
        status.add("Auxiliary1 Flag", rcvr_status.aux1_status_event_flag?"true":"false");
        status.add("Auxiliary2 Flag", rcvr_status.aux2_status_event_flag?"true":"false");
        status.add("Auxiliary3 Flag", rcvr_status.aux3_status_event_flag?"true":"false");
        NODELET_WARN("Novatel status code: %d", rcvr_status.original_status_code);
        status.summary(level, msg);
      }
    }

    void SyncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

      if (last_sync_ == ros::TIME_MIN)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No Sync");
        return;
      }
      else if (last_sync_ < ros::Time::now() - ros::Duration(10))
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Sync Stale");
        NODELET_ERROR("GPS time synchronization is stale.");
      }

      status.add("Last Sync", last_sync_);
      status.add("Mean Offset", stats::mean(offset_stats_));
      status.add("Mean Offset (rolling)", stats::rolling_mean(rolling_offset_));
      status.add("Offset Variance", stats::variance(offset_stats_));
      status.add("Min Offset", stats::min(offset_stats_));
      status.add("Max Offset", stats::max(offset_stats_));
    }

    void DeviceDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

      if (device_errors_ > 0)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Device Errors");
      }
      else if (device_interrupts_ > 0)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Device Interrupts");
        NODELET_WARN("device interrupts detected <%s:%s>: %d",
            connection_type_.c_str(), device_.c_str(), device_interrupts_);
      }
      else if (device_timeouts_)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Device Timeouts");
        NODELET_WARN("device timeouts detected <%s:%s>: %d",
            connection_type_.c_str(), device_.c_str(), device_timeouts_);
      }

      status.add("Errors", device_errors_);
      status.add("Interrupts", device_interrupts_);
      status.add("Timeouts", device_timeouts_);

      device_timeouts_ = 0;
      device_interrupts_ = 0;
      device_errors_ = 0;
    }

    void GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

      if (gps_parse_failures_ > 0)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Parse Failures");
        NODELET_WARN("gps parse failures detected <%s>: %d",
            hw_id_.c_str(), gps_parse_failures_);
      }

      status.add("Parse Failures", gps_parse_failures_);
      status.add("Insufficient Data Warnings", gps_insufficient_data_warnings_);

      gps_parse_failures_ = 0;
      gps_insufficient_data_warnings_ = 0;
    }

    void DataDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

      double period = diagnostic_updater_.getPeriod();
      double measured_rate = measurement_count_ / period;

      if (measured_rate < 0.5 * expected_rate_)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Insufficient Data Rate");
        NODELET_ERROR("insufficient data rate <%s>: %lf < %lf",
            hw_id_.c_str(), measured_rate, expected_rate_);
      }
      else if (measured_rate < 0.95 * expected_rate_)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Insufficient Data Rate");
        NODELET_WARN("insufficient data rate <%s>: %lf < %lf",
            hw_id_.c_str(), measured_rate, expected_rate_);
      }

      status.add("Measurement Rate (Hz)", measured_rate);

      measurement_count_ = 0;
    }

    void RateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status)
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal Publish Rate");

      double elapsed = (ros::Time::now() - last_published_).toSec();
      bool gap_detected = false;
      if (elapsed > 2.0 / expected_rate_)
      {
        publish_rate_warnings_++;
        gap_detected = true;
      }

      if (publish_rate_warnings_ > 1 || gap_detected)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Insufficient Publish Rate");
        NODELET_WARN("publish rate failures detected <%s>: %d",
            hw_id_.c_str(), publish_rate_warnings_);
      }

      status.add("Warnings", publish_rate_warnings_);

      publish_rate_warnings_ = 0;
    }
  };
}

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    novatel_oem628,
    novatel_gps_nodelet,
    novatel_oem628::NovatelGpsNodelet,
    nodelet::Nodelet)
