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

/**
 * \file
 *
 * This nodelet is a driver for NovAtel GNSS receivers. It publishes standard
 * ROS gps_msgs/GPSFix and sensor_msgs/NavSatFix messages as well as NMEA
 * logs and NovAtel logs.
 *
 * <b>Topics Subscribed:</b>
 *
 * \e gps_sync <tt>std_msgs/Time<tt> - Timestamped sync pulses
 *    from a DIO module (optional). These are used to improve the accuracy of
 *    the time stamps of the messages published.
 *
 * <b>Topics Published:</b>
 *
 * \e gps <tt>gps_msgs/GPSFix</tt> - GPS data for navigation
 * \e corrimudata <tt>novatel_gps_message/NovatelCorrectedImuData</tt> - Raw
 *    Novatel IMU data. (only published if `publish_imu_messages` is set `true`)
 * \e gppga <tt>novatel_gps_driver/Gpgga</tt> - Raw GPGGA data for debugging (only
 *    published if `publish_nmea_messages` is set `true`)
 * \e gpgsa <tt>novatel_gps_msgs/Gpgsa</tt> - Raw GPGSA data for debugging (only
 *    published if `publish_gpgsa` is set `true`)
 * \e gprmc <tt>novatel_gps_msgs/Gprmc</tt> - Raw GPRMC data for debugging (only
 *    published if `publish_nmea_messages` is set `true`)
 * \e bestpos <tt>novatel_gps_msgs/NovatelPosition</tt> - High fidelity Novatel-
 *    specific position and receiver status data. (only published if
 *    `publish_novatel_positions` is set `true`)
 * \e bestutm <tt>novatel_gps_msgs/NovatelUtmPosition</tt> - High fidelity Novatel-
 *    specific position in UTM coordinates and receiver status data. (only published
 *    if `publish_novatel_utm_positions` is set `true`)
 * \e bestvel <tt>novatel_gps_msgs/NovatelVelocity</tt> - High fidelity Novatel-
 *    specific velocity and receiver status data. (only published if
 *    `publish_novatel_velocity` is set `true`)
 * \e psrdop2 <tt>novatel_gps_msgs/Psrdop2</tt> - Pseudorange Dilution of Precision
 *    measurements. (only published if `publish_novatel_psrdop2` is set `true`)
 * \e range <tt>novatel_gps_msgs/Range</tt> - Satellite ranging information
 *    (only published if `publish_range_messages` is set `true`)
 * \e time <tt>novatel_gps_msgs/Time</tt> - Novatel-specific time data. (Only
 *    published if `publish_time` is set `true`.)
 * \e time_reference <tt>sensor_msgs/TimeReference</tt> - Generic time reference
 *    messages for time syncrhonization. (Only published if `publish_time_reference`
 *    is set `true`.)
 * \e trackstat <tt>novatel_gps_msgs/Trackstat</tt> - Novatel-specific trackstat
 *    data at 1 Hz. (Only published if `publish_trackstat` is set `true`.)
 *
 * <b>Services:</b>
 *
 * \e freset <tt>novatel_gps_msgs/NovatelFRESET</tt> - Sends a freset message to the
 *    device with the specified target string to reset. By default does
 *    FRESET standard
 *
 * <b>Parameters:</b>
 *
 * \e connection_type <tt>str</tt> - "serial", "udp", or "tcp" as appropriate
 *    for the Novatel device connected. ["serial"]
 * \e device <tt>str</tt> - The path to the device, e.g. /dev/ttyUSB0 for
 *    serial connections or "192.168.1.10:3001" for IP.
 *    [""]
 * \e frame_id <tt>str</tt> - The TF frame ID to set in all published message
 *    headers. [""]
 * \e imu_rate <tt>dbl</tt> - Rate at which to publish sensor_msgs/Imu messages.
 *    [100.0]
 * \e imu_sample_rate <tt>dbl</tt> - Rate at which the device internally samples
 *    its IMU. [200.0]
 * \e polling_period <tt>dbl</tt> - The number of seconds in between messages
 *    requested from the GPS. (Does not affect time messages) [0.05]
 * \e publish_diagnostics <tt>bool</tt> - If set true, the driver publishes
 *    ROS diagnostics [true]
 * \e publish_clocksteering <tt>bool</tt> - If set to true, the driver publishes
 *    Novatel ClockSteering messages [false]
 * \e publish_imu_messages <tt>boot</tt> - If set true, the driver publishes
 *    Novatel CorrImuData, InsPva, InsPvax, InsStdev, and sensor_msgs/Imu messages [false]
 * \e publish_gpgsa <tt>bool</tt> - If set true, the driver requests GPGSA
 *    messages from the device at 20 Hz and publishes them on `gpgsa`
 * \e publish_nmea_messages <tt>bool</tt> - If set true, the driver publishes
 *    GPGGA and GPRMC messages (see Topics Published) [false]
 * \e publish_novatel_messages <tt>bool</tt> - If set true, the driver
 *    publishes Novatel Bestpos messages (see Topics Published); even if
 *    this is false, these logs will still be requested from the receiver [false]
 * \e publish_novatel_psrdop2 <tt>bool</tt> - If set true, the driver
 *    published Novatel PSRDOP2 messages (see Topics Published); data from this
 *    message will be used to filled in DoP values in gps_msgs/GPSFix messages.
 *    Note that this topic is only published when the values changesages [false]
 * \e publish_novatel_velocity <tt>bool</tt> - If set true, the driver
 *    publishes Novatel Bestvel messages (see Topics Published); data from
 *    these messages will be used to fill in track/speed fields in
 *    gps_msgs/GPSFix messages [true]
 * \e publish_range_messages <tt>bool</tt> - If set true, the driver
 *    publishes Novatel RANGE messages [false]
 * \e publish_sync_diagnostic <tt>bool</tt> - If true, publish a Sync diagnostic.
 *    This is ignored if publish_diagnostics is false. [true]
 * \e publish_time_messages <tt>bool</tt> - If set true, the driver publishes Novatel
 *    Time messages (see Topics Published) [false]
 * \e publish_time_reference <tt>bool</tt> - If set true, the driver publishes
 *    sensor_msgs/msg/TimeReference messages (see Topics Published) [false]
 * \e publish_trackstat <tt>bool</tt> - If set true, the driver publishes
 *    Novatel Trackstat messages (see Topics Published) [false]
 * \e reconnect_delay_s <tt>bool</t> - If the driver is disconnected from the
 *    device, how long (in seconds) to wait between reconnect attempts. [0.5]
 * \e use_binary_messages <tt>bool</tt> - If set true, the driver requests
 *    binary NovAtel messages from the device; if false, it requests ASCII
 *    messages.  [false]
 * \e wait_for_sync <tt>bool</tt> - Wait for both BESTPOS and BESTVEL
 *    logs to arrive before pushing GPSFixes.  This has no effect if
 *    publish_novatel_velocity is false.  [true]
 * \e span_frame_to_ros_frame <tt>bool</tt> - Translate the SPAN coordinate
 *    frame to a ROS coordinate frame using the VEHICLEBODYROTATION and
 *    APPLYVEHICLEBODYROTATION commands. [false]
 */

#ifndef NOVATEL_GPS_DRIVER_NOVATEL_GPS_NODE_H
#define NOVATEL_GPS_DRIVER_NOVATEL_GPS_NODE_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <novatel_gps_driver/novatel_gps.h>

#include <novatel_gps_msgs/msg/clock_steering.hpp>
#include <novatel_gps_msgs/srv/novatel_freset.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <swri_roscpp/subscriber.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>

using TimeParserMsgT = novatel_gps_driver::TimeParser::MessageType;

namespace novatel_gps_driver
{
  class NovatelGpsNode : public rclcpp::Node
  {
  public:
    explicit NovatelGpsNode(const rclcpp::NodeOptions& options);

    ~NovatelGpsNode() override;

    void SyncCallback(const builtin_interfaces::msg::Time::ConstSharedPtr& sync);

    /**
     * Main spin loop connects to device, then reads data from it and publishes
     * messages.
     */
    void Spin();

  private:
    // Parameters
    /// The device identifier e.g. /dev/ttyUSB0
    std::string device_;
    /// The connection type, ("serial", "tcp", or "udp")
    std::string connection_type_;
    /// The baud rate used for serial connection
    int32_t serial_baud_;
    double polling_period_;
    bool publish_gpgsa_;
    bool publish_gpgsv_;
    bool publish_gphdt_;
    /// The rate at which IMU measurements will be published, in Hz
    double imu_rate_;
    /// How frequently the device samples the IMU, in Hz
    double imu_sample_rate_;
    bool span_frame_to_ros_frame_;
    bool publish_clock_steering_;
    bool publish_imu_messages_;
    bool publish_novatel_positions_;
    bool publish_novatel_xyz_positions_;
    bool publish_novatel_utm_positions_;
    bool publish_novatel_velocity_;
    bool publish_novatel_heading2_;
    bool publish_novatel_dual_antenna_heading_;
    bool publish_novatel_psrdop2_;
    bool publish_nmea_messages_;
    bool publish_range_messages_;
    bool publish_time_messages_;
    bool publish_time_reference_;
    bool publish_trackstat_;
    bool publish_diagnostics_;
    bool publish_sync_diagnostic_;
    bool publish_invalid_gpsfix_;
    double reconnect_delay_s_;
    bool use_binary_messages_;

    rclcpp::Publisher<novatel_gps_msgs::msg::ClockSteering>::SharedPtr clocksteering_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Inscov>::SharedPtr inscov_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Inspva>::SharedPtr inspva_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Inspvax>::SharedPtr inspvax_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Insstdev>::SharedPtr insstdev_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelCorrectedImuData>::SharedPtr novatel_imu_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelPosition>::SharedPtr novatel_position_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelXYZ>::SharedPtr novatel_xyz_position_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelUtmPosition>::SharedPtr novatel_utm_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelVelocity>::SharedPtr novatel_velocity_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelHeading2>::SharedPtr novatel_heading2_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelDualAntennaHeading>::SharedPtr novatel_dual_antenna_heading_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::NovatelPsrdop2>::SharedPtr novatel_psrdop2_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Gpgga>::SharedPtr gpgga_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Gpgsv>::SharedPtr gpgsv_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Gpgsa>::SharedPtr gpgsa_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Gphdt>::SharedPtr gphdt_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Gprmc>::SharedPtr gprmc_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Range>::SharedPtr range_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Time>::SharedPtr time_pub_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
    rclcpp::Publisher<novatel_gps_msgs::msg::Trackstat>::SharedPtr trackstat_pub_;

    rclcpp::Service<novatel_gps_msgs::srv::NovatelFRESET>::SharedPtr reset_service_;

    NovatelGps::ConnectionType connection_;
    NovatelGps gps_;

    boost::thread thread_;
    boost::mutex mutex_;

    /// Subscriber to listen for sync times from a DIO
    swri::Subscriber sync_sub_;
    rclcpp::Time last_sync_;
    /// Buffer of sync message time stamps
    boost::circular_buffer<rclcpp::Time> sync_times_;
    /// Buffer of gps message time stamps
    boost::circular_buffer<rclcpp::Time> msg_times_;
    /// Stats on time offset
    boost::accumulators::accumulator_set<float, boost::accumulators::stats<
        boost::accumulators::tag::max,
        boost::accumulators::tag::min,
        boost::accumulators::tag::mean,
        boost::accumulators::tag::variance> > offset_stats_;
    /// Rolling mean of time offset
    boost::accumulators::accumulator_set<float, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> > rolling_offset_;

    // ROS diagnostics
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
    rclcpp::Time last_published_;
    novatel_gps_msgs::msg::NovatelPosition::SharedPtr last_novatel_position_;

    std::string imu_frame_id_;
    std::string frame_id_;

    /**
     * @brief Service request to reset the gps through FRESET
     */
    bool resetService(std::shared_ptr<rmw_request_id_t> request_header,
                      novatel_gps_msgs::srv::NovatelFRESET::Request::SharedPtr req,
                      novatel_gps_msgs::srv::NovatelFRESET::Response::SharedPtr res);

    /**
     * @brief Reads data from the device and publishes any parsed messages.
     *
     * Note that when reading from the device, this will block until data
     * is available.
     */
    void CheckDeviceForData();

    sensor_msgs::msg::NavSatFix::UniquePtr ConvertGpsFixToNavSatFix(const gps_msgs::msg::GPSFix::UniquePtr& msg);

    /**
     * Updates the time sync offsets by matching up timesync messages to gps
     * messages and calculating the time offset between them.
     */
    void CalculateTimeSync();

    void FixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

    void SyncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

    void DeviceDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

    void GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

    void DataDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

    void RateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

    rclcpp::Time NovatelTimeToLocalTime(const TimeParserMsgT & utc_time);
  };
}

#endif //NOVATEL_GPS_DRIVER_NOVATEL_GPS_NODE_H
