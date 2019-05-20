/*
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
 */

#include <gtest/gtest.h>
#include "nodelets/novatel_gps_nodelet.cpp"

TEST(NovatelTest, DriverStatus)
{

    status_gps = cav_msgs::DriverStatus::OFF;
    publish_status(const ros::TimerEvent&);
    EXPECT_EQ(cav_msgs::DriverStatus::OFF, status_.status);
    status_gps = cav_msgs::DriverStatus::OPERATIONAL;
    publish_status(const ros::TimerEvent&);
    EXPECT_EQ(cav_msgs::DriverStatus::OPERATIONAL, status_.status);
    status_gps = cav_msgs::DriverStatus::FAULT;
    publish_status(const ros::TimerEvent&);
    EXPECT_EQ(cav_msgs::DriverStatus::FAULT, status_.status);
    status_gps = cav_msgs::DriverStatus::DEGRADED;
    publish_status(const ros::TimerEvent&);
    EXPECT_EQ(cav_msgs::DriverStatus::DEGRADED, status_.status);
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
