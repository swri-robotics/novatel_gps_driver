// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <ros/ros.h>
#include <ros/package.h>

TEST(NovatelGpsTestSuite, testGpsFixParsing)
{
  novatel_gps_driver::NovatelGps gps;

  std::string path = ros::package::getPath("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/gpgga-gprmc-bestpos.pcap", novatel_gps_driver::NovatelGps::PCAP));

  std::vector<gps_common::GPSFixPtr> fix_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<gps_common::GPSFixPtr> tmp_messages;
    gps.GetFixMessages(tmp_messages);
    fix_messages.insert(fix_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  ASSERT_EQ(40, fix_messages.size());
}

TEST(NovatelGpsTestSuite, testCorrImuDataParsing)
{
  novatel_gps_driver::NovatelGps gps;

  std::string path = ros::package::getPath("novatel_gps_driver");
  ASSERT_TRUE(gps.Connect(path + "/test/corrimudata.pcap", novatel_gps_driver::NovatelGps::PCAP));

  std::vector<novatel_gps_msgs::NovatelCorrectedImuDataPtr> imu_messages;

  while (gps.IsConnected() && gps.ProcessData() == novatel_gps_driver::NovatelGps::READ_SUCCESS)
  {
    std::vector<novatel_gps_msgs::NovatelCorrectedImuDataPtr> tmp_messages;
    gps.GetNovatelCorrectedImuData(tmp_messages);
    imu_messages.insert(imu_messages.end(), tmp_messages.begin(), tmp_messages.end());
  }

  ASSERT_EQ(26, imu_messages.size());

  novatel_gps_msgs::NovatelCorrectedImuDataPtr msg = imu_messages.front();
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
  ros::init(argc, argv, "novatel_gps_test_suite", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}