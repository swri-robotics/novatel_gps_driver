/*###Note: gtest is not currently functional due to the nodelet integration issue.
 * Copyright (C) 2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.


#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>
#include "../src/nodelets/novatel_gps_nodelet.cpp"


using ::testing::A;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::Unused;

class MockPublisher : public ros::Publisher {
  public:
    MOCK_METHOD2(publish, void(const cav_msgs::DriverStatus&));
    ~MockPublisher() {};
};

TEST(NovatelTest, DriverStatus)
{   const ros::TimerEvent event;
    novatel_gps_driver::NovatelGpsNodelet stat;
    stat.status_gps_ = cav_msgs::DriverStatus::OFF;
    //stat.publish_status(event);
    //EXPECT_EQ(cav_msgs::DriverStatus::OFF, stat.status_.status);
    stat.status_gps_ = cav_msgs::DriverStatus::OPERATIONAL;
    //stat.publish_status(event);
    //EXPECT_EQ(cav_msgs::DriverStatus::OPERATIONAL, stat.status_.status);
    stat.status_gps_ = cav_msgs::DriverStatus::FAULT;
    //stat.publish_status(event);
    //EXPECT_EQ(cav_msgs::DriverStatus::FAULT, stat.status_.status);
    stat.status_gps_ = cav_msgs::DriverStatus::DEGRADED;
    //stat.publish_status(event);
    //EXPECT_EQ(cav_msgs::DriverStatus::DEGRADED, stat.status_.status);
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleMock(&argc, argv);
    ros::init(argc, argv, "novatel_gtest", ros::init_options::AnonymousName);
    return RUN_ALL_TESTS();
}
*/
