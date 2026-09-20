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

#include <novatel_gps_driver/novatel_gps.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <ament_index_cpp/get_package_prefix.hpp>
// ament_index_cpp::get_package_prefix changed from returning std::string to
// taking an output std::filesystem::path& parameter in ament_index_cpp 1.13.0.
#include <ament_index_cpp/version.h>
#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 0)
#include <filesystem>
#endif

#include <rclcpp/rclcpp.hpp>

namespace
{
// The values test/make_imu_sync_pcaps.py writes into the synthetic captures.
// The rate and acceleration fields of CORRIMUDATA are increments accumulated over
// each logging interval, so the driver divides them by that interval.
constexpr double IMU_LOG_RATE_HZ = 100.0;
constexpr double PITCH_RATE = 0.001;          // about the SPAN x axis
constexpr double ROLL_RATE = 0.002;           // about the SPAN y axis
constexpr double YAW_RATE = 0.003;            // about the SPAN z axis
constexpr double LATERAL_ACC = 0.01;          // along the SPAN x axis
constexpr double LONGITUDINAL_ACC = 0.02;     // along the SPAN y axis
constexpr double VERTICAL_ACC = 0.03;         // along the SPAN z axis
constexpr double ROLL_DEG = 1.0;              // INSPVA attitude
constexpr double PITCH_DEG = 2.0;
constexpr double AZIMUTH_DEG = 3.0;           // clockwise from North
constexpr double ROLL_DEV_DEG = 1.0;          // INSSTDEV standard deviations
constexpr double PITCH_DEV_DEG = 2.0;
constexpr double AZIMUTH_DEV_DEG = 3.0;
constexpr double ROLL_VAR_DEG2 = 0.25;        // INSCOV variances
constexpr double PITCH_VAR_DEG2 = 1.0;
constexpr double AZIMUTH_VAR_DEG2 = 2.25;
constexpr double BESTVEL_TRACK_DEG = 45.0;    // deliberately different from INSPVAX_AZIMUTH_DEG
constexpr double INSPVAX_AZIMUTH_DEG = 270.0; // so the two are easy to tell apart in a test
constexpr double BESTVEL_HORIZONTAL_SPEED = 0.05;  // vertical speed is 0
constexpr size_t FIX_COUNT = 40;              // BESTPOS logs in the bestpos-bestvel-* sync captures
constexpr size_t DROPPED_BESTVEL_INDEX = 10;  // the epoch missing its BESTVEL in bestpos-bestvel-dropped

constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;

std::string GetPackagePrefix(const std::string & package_name)
{
#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 0)
  std::filesystem::path path;
  ament_index_cpp::get_package_prefix(package_name, path);
  return path.string();
#else
  return ament_index_cpp::get_package_prefix(package_name);
#endif
}
}  // namespace

class NovatelGpsTestSuite : public ::testing::Test, public rclcpp::Node
{
public:
  explicit NovatelGpsTestSuite() :
    rclcpp::Node("novatel_gps_test_suite")
  {}
protected:

};

TEST_F(NovatelGpsTestSuite, testGpsFixParsing)
{
  novatel_gps_driver::NovatelGps gps(*this);
  gps.wait_for_sync_ = true;

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/bestpos-bestvel-psrdop2-sync.pcap",
      novatel_gps_driver::NovatelGps::PCAP));

  std::vector<gps_msgs::msg::GPSFix::UniquePtr> fix_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<gps_msgs::msg::GPSFix::UniquePtr> tmp_messages;
    gps.GetFixMessages(tmp_messages);

    std::move(std::make_move_iterator(tmp_messages.begin()),
        std::make_move_iterator(tmp_messages.end()),
        std::back_inserter(fix_messages));
  }

  // One fix per BESTPOS/BESTVEL pair; the capture holds 33 of each.
  ASSERT_EQ(33, fix_messages.size());

  // Every BESTPOS in this capture has a BESTVEL logged at the same time, the first
  // one included; speed combines the horizontal and vertical components.
  EXPECT_DOUBLE_EQ(fix_messages[0]->time, 412623.4);
  EXPECT_DOUBLE_EQ(fix_messages[0]->speed, 0.025154091375098837);
  EXPECT_DOUBLE_EQ(fix_messages[0]->track, 56.304537721880898);

  EXPECT_DOUBLE_EQ(fix_messages[1]->latitude, 29.443917634921949);
  EXPECT_DOUBLE_EQ(fix_messages[1]->longitude, -98.614755510637181);
  EXPECT_DOUBLE_EQ(fix_messages[1]->speed, 0.041456376659522925);
  EXPECT_DOUBLE_EQ(fix_messages[1]->track, 135.51629763185957);
  EXPECT_DOUBLE_EQ(fix_messages[1]->gdop, 1.9980000257492065);
}

// BESTVEL's track_ground is derived from Doppler/carrier-phase velocity and gets
// noisy as ground speed approaches zero. INSPVAX's azimuth is the SPAN filter's
// true direction of travel and doesn't have that problem, so GetFixMessages()
// should prefer it over track_ground once the INS solution is good.
//
// Regression test for https://github.com/swri-robotics/novatel_gps_driver/issues/101.
TEST_F(NovatelGpsTestSuite, testGpsFixTrackPrefersInspvaxAzimuthOverBestvel)
{
  novatel_gps_driver::NovatelGps gps(*this);
  gps.wait_for_sync_ = true;

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/bestpos-bestvel-inspvax-sync.pcap",
      novatel_gps_driver::NovatelGps::PCAP));

  std::vector<gps_msgs::msg::GPSFix::UniquePtr> fix_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<gps_msgs::msg::GPSFix::UniquePtr> tmp_messages;
    gps.GetFixMessages(tmp_messages);

    std::move(std::make_move_iterator(tmp_messages.begin()),
        std::make_move_iterator(tmp_messages.end()),
        std::back_inserter(fix_messages));
  }

  ASSERT_EQ(1, fix_messages.size());

  // Not BESTVEL_TRACK_DEG: proves BESTVEL's track_ground was overridden rather
  // than just never having been set.
  EXPECT_DOUBLE_EQ(fix_messages.front()->track, INSPVAX_AZIMUTH_DEG);
}

// Replays a capture of BESTPOS and BESTVEL logs from test/make_imu_sync_pcaps.py
// and collects every GPSFix that GetFixMessages() produces from it.
//
// Regression tests for https://github.com/swri-robotics/novatel_gps_driver/issues/2,
// where GetFixMessages() waited forever for a BESTPOS's matching BESTVEL, so GPSFix
// output stopped entirely if BESTVEL lagged farther behind than the sync buffer held.
static std::vector<gps_msgs::msg::GPSFix::UniquePtr> ReplayFixCapture(
    rclcpp::Node& node,
    const std::string& capture,
    double sync_timeout = 1.0)
{
  novatel_gps_driver::NovatelGps gps(node);
  gps.wait_for_sync_ = true;
  gps.gpsfix_sync_timeout_ = sync_timeout;

  std::string path = GetPackagePrefix("novatel_gps_driver");
  EXPECT_TRUE(gps.Connect(path + "/test/" + capture, novatel_gps_driver::NovatelGps::PCAP));

  std::vector<gps_msgs::msg::GPSFix::UniquePtr> fix_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<gps_msgs::msg::GPSFix::UniquePtr> tmp_messages;
    gps.GetFixMessages(tmp_messages);

    std::move(std::make_move_iterator(tmp_messages.begin()),
        std::make_move_iterator(tmp_messages.end()),
        std::back_inserter(fix_messages));
  }

  return fix_messages;
}

static size_t CountSyncedFixes(const std::vector<gps_msgs::msg::GPSFix::UniquePtr>& fix_messages)
{
  return std::count_if(fix_messages.begin(), fix_messages.end(),
      [](const gps_msgs::msg::GPSFix::UniquePtr& fix) { return fix->track == BESTVEL_TRACK_DEG; });
}

// BESTVEL 5 epochs (0.25 s) behind BESTPOS is within the default timeout, so every
// fix should still wait for and get its speed & track.
TEST_F(NovatelGpsTestSuite, testGpsFixWaitsForLaggingBestvel)
{
  auto fix_messages = ReplayFixCapture(*this, "bestpos-bestvel-lag5.pcap");

  ASSERT_EQ(FIX_COUNT, fix_messages.size());
  EXPECT_EQ(FIX_COUNT, CountSyncedFixes(fix_messages));
  EXPECT_DOUBLE_EQ(fix_messages.front()->speed, BESTVEL_HORIZONTAL_SPEED);
}

// With a shorter timeout, a fix stops waiting once a BESTPOS 3 epochs (0.15 s) newer
// has arrived.  Only the last 3 BESTPOS are still waiting when the trailing BESTVELs
// arrive at the end of the capture.
TEST_F(NovatelGpsTestSuite, testGpsFixSyncTimeout)
{
  auto fix_messages = ReplayFixCapture(*this, "bestpos-bestvel-lag5.pcap", 0.12);

  ASSERT_EQ(FIX_COUNT, fix_messages.size());
  EXPECT_EQ(3, CountSyncedFixes(fix_messages));
  EXPECT_TRUE(std::isnan(fix_messages.front()->speed));
  EXPECT_TRUE(std::isnan(fix_messages.front()->track));
  EXPECT_DOUBLE_EQ(fix_messages.back()->speed, BESTVEL_HORIZONTAL_SPEED);
}

// BESTVEL 15 epochs behind BESTPOS is more than the sync buffer holds.  This used to
// stop GPSFix output entirely; now each BESTPOS is published without speed & track
// before the buffer would discard it.
TEST_F(NovatelGpsTestSuite, testGpsFixPublishedWhenBestvelLagsPastSyncBuffer)
{
  auto fix_messages = ReplayFixCapture(*this, "bestpos-bestvel-lag15.pcap");

  ASSERT_EQ(FIX_COUNT, fix_messages.size());
  for (size_t i = 1; i < fix_messages.size(); i++)
  {
    EXPECT_GT(fix_messages[i]->time, fix_messages[i - 1]->time);
  }
  EXPECT_TRUE(std::isnan(fix_messages.front()->speed));
  EXPECT_TRUE(std::isnan(fix_messages.front()->track));
}

// A missing BESTVEL shouldn't hold up the fixes after it: once a newer BESTVEL has
// arrived, the matching one never will, so that fix is published without it.
TEST_F(NovatelGpsTestSuite, testGpsFixPublishedWhenBestvelDropped)
{
  auto fix_messages = ReplayFixCapture(*this, "bestpos-bestvel-dropped.pcap");

  ASSERT_EQ(FIX_COUNT, fix_messages.size());
  EXPECT_EQ(FIX_COUNT - 1, CountSyncedFixes(fix_messages));
  EXPECT_TRUE(std::isnan(fix_messages[DROPPED_BESTVEL_INDEX]->speed));
  EXPECT_TRUE(std::isnan(fix_messages[DROPPED_BESTVEL_INDEX]->track));
  EXPECT_DOUBLE_EQ(fix_messages[DROPPED_BESTVEL_INDEX + 1]->speed, BESTVEL_HORIZONTAL_SPEED);
}

// Replays ASCII and binary RAWDMI and INSUPDATESTATUS logs from
// test/make_imu_sync_pcaps.py and checks that both formats of each reach their buffer.
//
// https://github.com/swri-robotics/novatel_gps_driver/issues/14
TEST_F(NovatelGpsTestSuite, testWheelSensorLogs)
{
  novatel_gps_driver::NovatelGps gps(*this);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/rawdmi-insupdatestatus.pcap",
      novatel_gps_driver::NovatelGps::PCAP));

  std::vector<novatel_gps_driver::RawDmiParser::MessageType> rawdmi_messages;
  std::vector<novatel_gps_driver::InsUpdateStatusParser::MessageType> insupdatestatus_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<novatel_gps_driver::RawDmiParser::MessageType> tmp_rawdmi;
    gps.GetRawDmiMessages(tmp_rawdmi);
    std::move(std::make_move_iterator(tmp_rawdmi.begin()),
        std::make_move_iterator(tmp_rawdmi.end()),
        std::back_inserter(rawdmi_messages));

    std::vector<novatel_gps_driver::InsUpdateStatusParser::MessageType> tmp_insupdatestatus;
    gps.GetInsUpdateStatusMessages(tmp_insupdatestatus);
    std::move(std::make_move_iterator(tmp_insupdatestatus.begin()),
        std::make_move_iterator(tmp_insupdatestatus.end()),
        std::back_inserter(insupdatestatus_messages));
  }

  ASSERT_EQ(2, rawdmi_messages.size());
  EXPECT_EQ(2297, rawdmi_messages[0]->dmi[0]);   // ASCII
  EXPECT_EQ(4096, rawdmi_messages[1]->dmi[0]);   // binary
  EXPECT_EQ(1u, rawdmi_messages[1]->mask);

  ASSERT_EQ(2, insupdatestatus_messages.size());
  EXPECT_EQ("INACTIVE", insupdatestatus_messages[0]->dmi_update_status);  // ASCII
  EXPECT_EQ("USED", insupdatestatus_messages[1]->dmi_update_status);      // binary
  EXPECT_EQ("INS_PSRSP", insupdatestatus_messages[1]->position_type);
}

// Replays RAWIMUX logs in ASCII and binary, and RAWIMUSX in binary, and checks
// that each reaches the RAWIMUX buffer.
//
// https://github.com/swri-robotics/novatel_gps_driver/issues/39
TEST_F(NovatelGpsTestSuite, testRawImuxLogs)
{
  novatel_gps_driver::NovatelGps gps(*this);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/rawimux.pcap", novatel_gps_driver::NovatelGps::PCAP));

  std::vector<novatel_gps_driver::RawImuxParser::MessageType> rawimux_messages;
  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<novatel_gps_driver::RawImuxParser::MessageType> tmp_messages;
    gps.GetRawImuxMessages(tmp_messages);
    std::move(std::make_move_iterator(tmp_messages.begin()),
        std::make_move_iterator(tmp_messages.end()),
        std::back_inserter(rawimux_messages));
  }

  ASSERT_EQ(6u, rawimux_messages.size());
  for (size_t i = 0; i < 3; i++)
  {
    // ASCII logs keep the name as the receiver sent it, like every other ASCII log.
    EXPECT_EQ("RAWIMUXA", rawimux_messages[i]->novatel_msg_header.message_name);
    EXPECT_EQ(41, rawimux_messages[i]->imu_type);
  }
  EXPECT_EQ("RAWIMUX", rawimux_messages[3]->novatel_msg_header.message_name);
  EXPECT_EQ("RAWIMUX", rawimux_messages[4]->novatel_msg_header.message_name);
  EXPECT_EQ("RAWIMUSX", rawimux_messages[5]->novatel_msg_header.message_name);
  for (size_t i = 3; i < 6; i++)
  {
    EXPECT_EQ(58, rawimux_messages[i]->imu_type);
    EXPECT_EQ(1, rawimux_messages[i]->z_acceleration);
    EXPECT_EQ(-6, rawimux_messages[i]->x_rotation);
  }
}

// Replays the NMEA capture from test/make_imu_sync_pcaps.py and checks that every
// sentence reaches the raw sentence buffer as it was sent, including the GPVTG the
// driver has no parser for, and that the NovAtel ASCII log in the same capture
// doesn't.
//
// https://github.com/swri-robotics/novatel_gps_driver/issues/92
TEST_F(NovatelGpsTestSuite, testNmeaSentencesRepublishedVerbatim)
{
  novatel_gps_driver::NovatelGps gps(*this);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/nmea-sentences.pcap", novatel_gps_driver::NovatelGps::PCAP));

  std::vector<nmea_msgs::msg::Sentence::UniquePtr> sentences;
  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<nmea_msgs::msg::Sentence::UniquePtr> tmp_sentences;
    gps.GetNmeaSentences(tmp_sentences);
    std::move(std::make_move_iterator(tmp_sentences.begin()),
        std::make_move_iterator(tmp_sentences.end()),
        std::back_inserter(sentences));
  }

  ASSERT_EQ(3u, sentences.size());
  EXPECT_EQ("$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60",
            sentences[0]->sentence);
  EXPECT_EQ("$GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20",
            sentences[1]->sentence);
  EXPECT_EQ("$GPVTG,172.516,T,155.295,M,0.049,N,0.090,K,D*2B", sentences[2]->sentence);
  for (const auto& sentence : sentences)
  {
    EXPECT_NE(rclcpp::Time(sentence->header.stamp), rclcpp::Time(0, 0, RCL_ROS_TIME));
  }
}

TEST_F(NovatelGpsTestSuite, testCorrImuDataParsing)
{
  novatel_gps_driver::NovatelGps gps(*this);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/corrimudata.pcap", novatel_gps_driver::NovatelGps::PCAP));

  std::vector<novatel_gps_driver::CorrImuDataParser::MessageType> imu_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<novatel_gps_driver::CorrImuDataParser::MessageType> tmp_messages;
    gps.GetNovatelCorrectedImuData(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  // The capture holds 29 CORRIMUDATA logs.
  ASSERT_EQ(29, imu_messages.size());

  novatel_gps_driver::CorrImuDataParser::MessageType msg = imu_messages.front();
  EXPECT_EQ(1820, msg->gps_week_num);
  EXPECT_DOUBLE_EQ(160205.899999999994, msg->gps_seconds);
  EXPECT_DOUBLE_EQ(0.0000039572689929003956, msg->pitch_rate);
  EXPECT_DOUBLE_EQ(0.0000028926313702935847, msg->roll_rate);
  EXPECT_DOUBLE_EQ(0.0000027924848999730557, msg->yaw_rate);
  EXPECT_DOUBLE_EQ(-0.00062560456243879322, msg->lateral_acceleration);
  EXPECT_DOUBLE_EQ(0.00034037959880710289, msg->longitudinal_acceleration);
  EXPECT_DOUBLE_EQ(-0.0000051257464089797534, msg->vertical_acceleration);
}

// Replays a capture containing paired corrected-IMU and INS position/velocity/attitude
// logs and checks that the two are combined into sensor_msgs/Imu messages.
//
// Regression test for https://github.com/swri-robotics/novatel_gps_driver/issues/127,
// where NovatelGps::GenerateImuMessages popped all four of its synchronization queues
// even though only two of them had supplied the messages being paired, so a receiver
// logging one variant crashed as soon as an IMU rate was known.
static void ReplayImuCapture(rclcpp::Node& node,
                             const std::string& capture,
                             std::vector<sensor_msgs::msg::Imu::SharedPtr>& imu_messages,
                             bool span_frame_to_ros_frame = false)
{
  novatel_gps_driver::NovatelGps gps(node);
  // Deprecated and ignored; passed through so a test can prove it changes nothing.
  gps.ApplyVehicleBodyRotation(span_frame_to_ros_frame);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/" + capture, novatel_gps_driver::NovatelGps::PCAP));

  imu_messages.clear();
  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<sensor_msgs::msg::Imu::SharedPtr> tmp_messages;
    gps.GetImuMessages(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  // The captures hold ten IMU/INS pairs, all within IMU_TOLERANCE_S of each other.
  // The first corrected-IMU log is dropped, because without an earlier one there's
  // no telling how much time its increments cover.
  ASSERT_EQ(9u, imu_messages.size());
}

static void ExpectSynchronizedImuMessages(rclcpp::Node& node, const std::string& capture)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  ReplayImuCapture(node, capture, imu_messages);
  ASSERT_FALSE(imu_messages.empty());

  sensor_msgs::msg::Imu::SharedPtr msg = imu_messages.front();

  // Attitude comes from the INS log, rotated into the ROS frame.
  EXPECT_NEAR(0.018342027500639145, msg->orientation.x, 1e-12);
  EXPECT_NEAR(-0.006653010553465184, msg->orientation.y, 1e-12);
  EXPECT_NEAR(0.68833400334019520, msg->orientation.z, 1e-12);
  EXPECT_NEAR(0.72513144141141830, msg->orientation.w, 1e-12);

  // Rates and accelerations come from the corrected IMU log, divided by its
  // logging interval and rotated from the SPAN vehicle frame into the ROS body frame.
  EXPECT_NEAR(ROLL_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.x, 1e-12);
  EXPECT_NEAR(-PITCH_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.y, 1e-12);
  EXPECT_NEAR(YAW_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.z, 1e-12);
  EXPECT_NEAR(LONGITUDINAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.x, 1e-12);
  EXPECT_NEAR(-LATERAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.y, 1e-12);
  EXPECT_NEAR(VERTICAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.z, 1e-12);

  // Orientation covariance comes from the INSSTDEV log at the head of the capture.
  EXPECT_NEAR(std::pow(ROLL_DEV_DEG * DEGREES_TO_RADIANS, 2),
              msg->orientation_covariance[0], 1e-12);
  EXPECT_NEAR(std::pow(PITCH_DEV_DEG * DEGREES_TO_RADIANS, 2),
              msg->orientation_covariance[4], 1e-12);
  EXPECT_NEAR(std::pow(AZIMUTH_DEV_DEG * DEGREES_TO_RADIANS, 2),
              msg->orientation_covariance[8], 1e-12);
}

// CORRIMUDATA holds the IMU samples accumulated over each logging interval, which
// the driver used to multiply by the IMU's sample rate instead of dividing by the
// interval.  A receiver also logs all zeros for an interval that caught no IMU
// sample, which the driver published as a rate of zero.
//
// Regression test for https://github.com/swri-robotics/novatel_gps_driver/issues/28.
TEST_F(NovatelGpsTestSuite, testImuRatesUseEachLogsInterval)
{
  novatel_gps_driver::NovatelGps gps(*this);
  // Twice the logging rate; this must not affect the scaling.
  gps.SetImuRate(2.0 * IMU_LOG_RATE_HZ, true);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/corrimudata-intervals.pcap", novatel_gps_driver::NovatelGps::PCAP));

  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<sensor_msgs::msg::Imu::SharedPtr> tmp_messages;
    gps.GetImuMessages(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  // Of the six CORRIMUDATA logs, the first has nothing before it to measure from,
  // one holds no IMU data, and one comes after a lost log, so only three produce
  // a sensor_msgs/Imu: one covering one interval, one covering two, and one
  // covering one.  All three should report the same rates.
  ASSERT_EQ(3u, imu_messages.size());
  for (const auto& msg : imu_messages)
  {
    EXPECT_NEAR(YAW_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.z, 1e-12);
    EXPECT_NEAR(ROLL_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.x, 1e-12);
    EXPECT_NEAR(VERTICAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.z, 1e-12);
  }
}

TEST_F(NovatelGpsTestSuite, testImuFromCorrImuDataAndInspva)
{
  ExpectSynchronizedImuMessages(*this, "corrimudata-inspva-sync.pcap");
}

TEST_F(NovatelGpsTestSuite, testImuFromCorrImusAndInspvas)
{
  ExpectSynchronizedImuMessages(*this, "corrimus-inspvas-sync.pcap");
}

// sensor_msgs/Imu is defined in the ROS body frame (REP 103: x forward, y left,
// z up), and NovatelGps::GenerateImuMessages already rotates the INS attitude
// into it -- it negates the SPAN pitch and azimuth when building the quaternion.
// The angular rates and accelerations it copies out of CORRIMUDATA get no such
// treatment, so a single sensor_msgs/Imu describes its orientation in one frame
// and its rates and accelerations in another.
//
// Per NovAtel's CORRIMUDATA documentation, PitchRate is "about x axis rotation",
// RollRate is "about y axis rotation", LateralAcc is "along x axis" and
// LongitudinalAcc is "along y axis" -- the SPAN vehicle frame is x right,
// y forward, z up.  Converting that to the ROS body frame is x_ros = y_span,
// y_ros = -x_span, z_ros = z_span.
//
// Reported in https://github.com/swri-robotics/novatel_gps_driver/issues/114.
TEST_F(NovatelGpsTestSuite, testImuVectorsUseTheRosBodyFrame)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  ReplayImuCapture(*this, "corrimudata-inspva-sync.pcap", imu_messages);
  ASSERT_FALSE(imu_messages.empty());

  sensor_msgs::msg::Imu::SharedPtr msg = imu_messages.front();

  // Roll is about the ROS x axis, so the SPAN roll rate belongs there.
  EXPECT_NEAR(ROLL_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.x, 1e-12);
  EXPECT_NEAR(-PITCH_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.y, 1e-12);
  EXPECT_NEAR(YAW_RATE * IMU_LOG_RATE_HZ, msg->angular_velocity.z, 1e-12);

  // ROS x points forward, which is where the longitudinal acceleration acts.
  EXPECT_NEAR(LONGITUDINAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.x, 1e-12);
  EXPECT_NEAR(-LATERAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.y, 1e-12);
  EXPECT_NEAR(VERTICAL_ACC * IMU_LOG_RATE_HZ, msg->linear_acceleration.z, 1e-12);
}

// The INSSTDEV branch of GenerateImuMessages fills orientation_covariance with
// std::pow(2, dev), which raises two to the standard deviation instead of
// squaring it, and it does so without converting NovAtel's degrees to the
// radians sensor_msgs/Imu is specified in.  It also puts the pitch deviation on
// the x axis and the roll deviation on the y axis, the opposite of the mapping
// the INSCOV branch below uses.
//
// Reported in https://github.com/swri-robotics/novatel_gps_driver/issues/114.
TEST_F(NovatelGpsTestSuite, testImuOrientationCovarianceFromInsstdev)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  ReplayImuCapture(*this, "corrimudata-inspva-sync.pcap", imu_messages);
  ASSERT_FALSE(imu_messages.empty());

  sensor_msgs::msg::Imu::SharedPtr msg = imu_messages.front();

  EXPECT_NEAR(std::pow(ROLL_DEV_DEG * DEGREES_TO_RADIANS, 2),
              msg->orientation_covariance[0], 1e-12);
  EXPECT_NEAR(std::pow(PITCH_DEV_DEG * DEGREES_TO_RADIANS, 2),
              msg->orientation_covariance[4], 1e-12);
  EXPECT_NEAR(std::pow(AZIMUTH_DEV_DEG * DEGREES_TO_RADIANS, 2),
              msg->orientation_covariance[8], 1e-12);
}

// GenerateImuMessages prefers INSCOV over INSSTDEV when both are logged, and the
// driver requests both, so this is the branch most receivers actually take.  It
// copies NovAtel's attitude covariance across verbatim; INSCOV reports it in
// deg^2, sensor_msgs/Imu wants rad^2.
//
// Reported in https://github.com/swri-robotics/novatel_gps_driver/issues/114.
TEST_F(NovatelGpsTestSuite, testImuOrientationCovarianceFromInscov)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  ReplayImuCapture(*this, "corrimudata-inspva-inscov.pcap", imu_messages);
  ASSERT_FALSE(imu_messages.empty());

  sensor_msgs::msg::Imu::SharedPtr msg = imu_messages.front();

  const double degrees2_to_radians2 = DEGREES_TO_RADIANS * DEGREES_TO_RADIANS;
  EXPECT_NEAR(ROLL_VAR_DEG2 * degrees2_to_radians2, msg->orientation_covariance[0], 1e-12);
  EXPECT_NEAR(PITCH_VAR_DEG2 * degrees2_to_radians2, msg->orientation_covariance[4], 1e-12);
  EXPECT_NEAR(AZIMUTH_VAR_DEG2 * degrees2_to_radians2, msg->orientation_covariance[8], 1e-12);

  // The off-diagonal terms of the covariance must survive the copy too.
  for (size_t i : {1u, 2u, 3u, 5u, 6u, 7u})
  {
    EXPECT_NEAR(0.0, msg->orientation_covariance[i], 1e-12) << "at index " << i;
  }
}

// span_frame_to_ros_frame is deprecated: the driver converts to the ROS frame
// itself and no longer sends VEHICLEBODYROTATION or APPLYVEHICLEBODYROTATION.
// Those commands could never do the whole job -- per the SPAN firmware reference
// they rotate only the INSPVA, INSPVAS, INSPVAX, INSATT, INSATTS and INSATTX
// logs, leaving CORRIMUDATA in the SPAN frame -- so setting the option must now
// make no difference at all to what comes out.
TEST_F(NovatelGpsTestSuite, testDeprecatedSpanFrameOptionIsIgnored)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> without_option;
  ReplayImuCapture(*this, "corrimudata-inspva-sync.pcap", without_option, false);
  ASSERT_FALSE(without_option.empty());

  std::vector<sensor_msgs::msg::Imu::SharedPtr> with_option;
  ReplayImuCapture(*this, "corrimudata-inspva-sync.pcap", with_option, true);
  ASSERT_EQ(without_option.size(), with_option.size());

  for (size_t i = 0; i < with_option.size(); i++)
  {
    const sensor_msgs::msg::Imu& a = *without_option[i];
    const sensor_msgs::msg::Imu& b = *with_option[i];

    EXPECT_DOUBLE_EQ(a.orientation.x, b.orientation.x) << "message " << i;
    EXPECT_DOUBLE_EQ(a.orientation.y, b.orientation.y) << "message " << i;
    EXPECT_DOUBLE_EQ(a.orientation.z, b.orientation.z) << "message " << i;
    EXPECT_DOUBLE_EQ(a.orientation.w, b.orientation.w) << "message " << i;
    EXPECT_DOUBLE_EQ(a.angular_velocity.x, b.angular_velocity.x) << "message " << i;
    EXPECT_DOUBLE_EQ(a.angular_velocity.y, b.angular_velocity.y) << "message " << i;
    EXPECT_DOUBLE_EQ(a.angular_velocity.z, b.angular_velocity.z) << "message " << i;
    EXPECT_DOUBLE_EQ(a.linear_acceleration.x, b.linear_acceleration.x) << "message " << i;
    EXPECT_DOUBLE_EQ(a.linear_acceleration.y, b.linear_acceleration.y) << "message " << i;
    EXPECT_DOUBLE_EQ(a.linear_acceleration.z, b.linear_acceleration.z) << "message " << i;
  }
}

// NovAtel measures azimuth clockwise from North; ROS measures yaw counter-clockwise
// from East, and its body frame points x forward where the SPAN vehicle frame points
// x right.  Both are the same quarter turn about z, and GenerateImuMessages used to
// apply neither, leaving the published heading 90 degrees off.
//
// Rather than restate the conversion, this rotates the body frame's forward axis by
// the published orientation and checks that the nose ends up pointing along the
// azimuth the receiver actually reported.
//
// Reported in https://github.com/swri-robotics/novatel_gps_driver/issues/114.
TEST_F(NovatelGpsTestSuite, testImuOrientationHeadingMatchesAzimuth)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  ReplayImuCapture(*this, "corrimudata-inspva-sync.pcap", imu_messages);
  ASSERT_FALSE(imu_messages.empty());

  const geometry_msgs::msg::Quaternion& q = imu_messages.front()->orientation;

  // The body frame's x axis, expressed in the ENU world frame: the first column of
  // the rotation matrix the quaternion stands for.
  const double east = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double north = 2.0 * (q.x * q.y + q.w * q.z);

  // A compass heading is measured clockwise from North, which is what azimuth is.
  const double heading_degrees = std::atan2(east, north) / DEGREES_TO_RADIANS;
  EXPECT_NEAR(AZIMUTH_DEG, heading_degrees, 1e-9);
}

// The write-only port RTCM corrections are sent to.
// https://github.com/swri-robotics/novatel_gps_driver/issues/97

TEST_F(NovatelGpsTestSuite, testCorrectionPortWritesWhatItIsGiven)
{
  // Stand in for a receiver listening on one of its IP ports.
  boost::asio::io_context io;
  boost::asio::ip::tcp::acceptor acceptor(
      io, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0));
  uint16_t port = acceptor.local_endpoint().port();

  novatel_gps_driver::NovatelGps gps(*this);
  EXPECT_FALSE(gps.IsCorrectionPortConnected());

  ASSERT_TRUE(gps.ConnectCorrectionPort("127.0.0.1:" + std::to_string(port),
                                        novatel_gps_driver::NovatelGps::TCP, 0));
  EXPECT_TRUE(gps.IsCorrectionPortConnected());

  boost::asio::ip::tcp::socket receiver(io);
  acceptor.accept(receiver);

  const std::vector<uint8_t> corrections = {0xD3, 0x00, 0x04, 0x4C, 0xE0, 0x00, 0x80, 0xED, 0xED, 0xD6};
  ASSERT_TRUE(gps.WriteCorrections(corrections));

  std::vector<uint8_t> received(corrections.size());
  boost::asio::read(receiver, boost::asio::buffer(received));
  EXPECT_EQ(corrections, received);

  gps.DisconnectCorrectionPort();
  EXPECT_FALSE(gps.IsCorrectionPortConnected());
}

TEST_F(NovatelGpsTestSuite, testCorrectionsAreNotWrittenWithoutAPort)
{
  novatel_gps_driver::NovatelGps gps(*this);

  EXPECT_FALSE(gps.IsCorrectionPortConnected());
  EXPECT_FALSE(gps.WriteCorrections({0xD3, 0x00, 0x00}));
}

TEST_F(NovatelGpsTestSuite, testCorrectionPortRejectsEndpointWithoutAHost)
{
  novatel_gps_driver::NovatelGps gps(*this);

  EXPECT_FALSE(gps.ConnectCorrectionPort("3003", novatel_gps_driver::NovatelGps::TCP, 0));
  EXPECT_FALSE(gps.IsCorrectionPortConnected());
  EXPECT_NE(gps.ErrorMsg().find("host"), std::string::npos);

  // A pcap file can't be written to.
  EXPECT_FALSE(gps.ConnectCorrectionPort("capture.pcap", novatel_gps_driver::NovatelGps::PCAP, 0));
  EXPECT_FALSE(gps.IsCorrectionPortConnected());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}