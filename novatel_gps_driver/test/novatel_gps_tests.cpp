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
// The rate and acceleration fields of CORRIMUDATA are increments over one IMU
// sample period (rad/sample and m/s/sample), so the driver scales them by the
// sample rate.
constexpr double IMU_SAMPLE_RATE_HZ = 100.0;
constexpr double PITCH_RATE = 0.001;          // about the SPAN x axis
constexpr double ROLL_RATE = 0.002;           // about the SPAN y axis
constexpr double YAW_RATE = 0.003;            // about the SPAN z axis
constexpr double LATERAL_ACC = 0.01;          // along the SPAN x axis
constexpr double LONGITUDINAL_ACC = 0.02;     // along the SPAN y axis
constexpr double VERTICAL_ACC = 0.03;         // along the SPAN z axis
constexpr double ROLL_DEV_DEG = 1.0;          // INSSTDEV standard deviations
constexpr double PITCH_DEV_DEG = 2.0;
constexpr double AZIMUTH_DEV_DEG = 3.0;
constexpr double ROLL_VAR_DEG2 = 0.25;        // INSCOV variances
constexpr double PITCH_VAR_DEG2 = 1.0;
constexpr double AZIMUTH_VAR_DEG2 = 2.25;

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

  ASSERT_EQ(22, fix_messages.size());

  EXPECT_DOUBLE_EQ(fix_messages.front()->latitude, 29.443917634921949);
  EXPECT_DOUBLE_EQ(fix_messages.front()->longitude, -98.614755510637181);
  EXPECT_DOUBLE_EQ(fix_messages.front()->speed, 0.041456376659522925);
  EXPECT_DOUBLE_EQ(fix_messages.front()->track, 135.51629763185957);
  EXPECT_DOUBLE_EQ(fix_messages.front()->gdop, 1.9980000257492065);
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

  ASSERT_EQ(26, imu_messages.size());

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
                             std::vector<sensor_msgs::msg::Imu::SharedPtr>& imu_messages)
{
  novatel_gps_driver::NovatelGps gps(node);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/" + capture, novatel_gps_driver::NovatelGps::PCAP));

  // The IMU rate is normally learned from the receiver's configuration; without it
  // GenerateImuMessages returns before it pairs anything up.  The captures carry
  // increments sampled at IMU_SAMPLE_RATE_HZ, which is what the driver multiplies
  // by to turn them into rates and accelerations.
  gps.SetImuRate(IMU_SAMPLE_RATE_HZ, true);

  imu_messages.clear();
  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<sensor_msgs::msg::Imu::SharedPtr> tmp_messages;
    gps.GetImuMessages(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  // The captures hold ten IMU/INS pairs, all within IMU_TOLERANCE_S of each other.
  ASSERT_EQ(10u, imu_messages.size());
}

static void ExpectSynchronizedImuMessages(rclcpp::Node& node, const std::string& capture)
{
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;
  ReplayImuCapture(node, capture, imu_messages);
  ASSERT_FALSE(imu_messages.empty());

  sensor_msgs::msg::Imu::SharedPtr msg = imu_messages.front();

  // Attitude comes from the INS log, rotated into the ROS frame.
  EXPECT_NEAR(0.0082653831487511844, msg->orientation.x, 1e-12);
  EXPECT_NEAR(-0.017674160904072977, msg->orientation.y, 1e-12);
  EXPECT_NEAR(-0.026019717990453804, msg->orientation.z, 1e-12);
  EXPECT_NEAR(0.99947100095672536, msg->orientation.w, 1e-12);

  // Rates and accelerations come from the corrected IMU log, scaled by the IMU rate.
  EXPECT_NEAR(0.1, msg->angular_velocity.x, 1e-12);
  EXPECT_NEAR(0.2, msg->angular_velocity.y, 1e-12);
  EXPECT_NEAR(0.3, msg->angular_velocity.z, 1e-12);
  EXPECT_NEAR(1.0, msg->linear_acceleration.x, 1e-12);
  EXPECT_NEAR(2.0, msg->linear_acceleration.y, 1e-12);
  EXPECT_NEAR(3.0, msg->linear_acceleration.z, 1e-12);

  // Orientation covariance comes from the INSSTDEV log at the head of the capture.
  EXPECT_DOUBLE_EQ(4.0, msg->orientation_covariance[0]);
  EXPECT_DOUBLE_EQ(2.0, msg->orientation_covariance[4]);
  EXPECT_DOUBLE_EQ(8.0, msg->orientation_covariance[8]);
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
  EXPECT_NEAR(ROLL_RATE * IMU_SAMPLE_RATE_HZ, msg->angular_velocity.x, 1e-12);
  EXPECT_NEAR(-PITCH_RATE * IMU_SAMPLE_RATE_HZ, msg->angular_velocity.y, 1e-12);
  EXPECT_NEAR(YAW_RATE * IMU_SAMPLE_RATE_HZ, msg->angular_velocity.z, 1e-12);

  // ROS x points forward, which is where the longitudinal acceleration acts.
  EXPECT_NEAR(LONGITUDINAL_ACC * IMU_SAMPLE_RATE_HZ, msg->linear_acceleration.x, 1e-12);
  EXPECT_NEAR(-LATERAL_ACC * IMU_SAMPLE_RATE_HZ, msg->linear_acceleration.y, 1e-12);
  EXPECT_NEAR(VERTICAL_ACC * IMU_SAMPLE_RATE_HZ, msg->linear_acceleration.z, 1e-12);
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}