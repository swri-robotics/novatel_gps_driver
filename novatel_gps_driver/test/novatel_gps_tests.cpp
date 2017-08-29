// *****************************************************************************
//
// Copyright (C) 2016 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
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