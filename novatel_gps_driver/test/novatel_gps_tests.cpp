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

#include <rclcpp/rclcpp.hpp>

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

  std::string path = ament_index_cpp::get_package_prefix("novatel_gps_driver");
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

  std::string path = ament_index_cpp::get_package_prefix("novatel_gps_driver");
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}