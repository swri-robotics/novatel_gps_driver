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
static void ExpectSynchronizedImuMessages(rclcpp::Node& node, const std::string& capture)
{
  novatel_gps_driver::NovatelGps gps(node);

  std::string path = GetPackagePrefix("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/" + capture, novatel_gps_driver::NovatelGps::PCAP));

  // The IMU rate is normally learned from the receiver's configuration; without it
  // GenerateImuMessages returns before it pairs anything up.
  gps.SetImuRate(100.0, true);

  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<sensor_msgs::msg::Imu::SharedPtr> tmp_messages;
    gps.GetImuMessages(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  // The capture holds ten IMU/INS pairs, all within IMU_TOLERANCE_S of each other.
  ASSERT_EQ(10u, imu_messages.size());

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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}