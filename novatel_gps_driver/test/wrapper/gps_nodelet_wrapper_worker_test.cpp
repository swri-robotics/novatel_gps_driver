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
#include <gmock/gmock.h>
#include <ros/ros.h>
#include <novatel_gps_driver/wrapper/gps_driver_wrapper.h>
#include <novatel_gps_driver/wrapper/novatel_gps_nodelet_wrapper_worker.h>
#include <novatel_gps_driver/wrapper/worker_config.h>

using ::testing::A;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::Unused;

using namespace novatel_gps_driver;

class MockPub : public GPSDriverWrapper 
{
  public:
    int fixCount = 0;
    int statusCount = 0;
    int alertCount = 0;
    int exceptionCount = 0;
    int shutdownCount = 0;
    cav_msgs::DriverStatus status;
    gps_common::GPSFix fix_msg;
    std::string name_;

    void publishFixFused(const gps_common::GPSFix& msg) override {
      fix_msg = msg;
      fixCount++;
    }

    void publishStatus(const cav_msgs::DriverStatus& msg) override {
      status=msg;
      statusCount++;
    }

    void publishAlert(const cav_msgs::SystemAlert& msg) override {
      alertCount++;
    }
    
    std::string getQualifiedName() override {
        return name_;
    }

    void setName(const std::string& name) {
      name_ = name;
    }
};

TEST(NovatelGPSWrapperWorkerTest, TestConstructor)
{   
  WorkerConfig config;
  config.gnss_timeout = ros::Duration(1.0);
  config.imu_timeout = ros::Duration(1.0);

  boost::shared_ptr<MockPub> pub;

  NovatelGPSNodeletWrapperWorker worker(config, pub);
}

TEST(NovatelGPSWrapperWorkerTest, TestTimerCallback)
{   
  WorkerConfig config;
  config.gnss_timeout = ros::Duration(1.0);
  config.imu_timeout = ros::Duration(1.0);

  boost::shared_ptr<MockPub> pub;
  pub.reset(new MockPub());
  pub->setName("MockPub");
  NovatelGPSNodeletWrapperWorker worker(config, pub);
  
  // No data received
  ros::TimerEvent event;
  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::OFF, pub->status.status);
  ASSERT_EQ(pub->getQualifiedName(), pub->status.name);
  ASSERT_EQ(true, pub->status.imu);
  ASSERT_EQ(true, pub->status.gnss);

  // Imu message
  ros::Time startTime(1.0);
  ros::Time::setNow(startTime); // Start time
  sensor_msgs::Imu msg;
  sensor_msgs::ImuConstPtr msg_ptr(new sensor_msgs::Imu(msg));
  
  worker.imuCallback(msg_ptr);

  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::OFF, pub->status.status);

  // GPS message
  novatel_gps_msgs::Inspvax gps_msg;
  novatel_gps_msgs::InspvaxConstPtr gps_msg_ptr(new novatel_gps_msgs::Inspvax(gps_msg));
  
  worker.inspvaxCallback(gps_msg_ptr);

  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::OPERATIONAL, pub->status.status);

  // Time passes
  ros::Time::setNow(startTime + ros::Duration(0.5));
  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::OPERATIONAL, pub->status.status);

  // GPS message
  gps_msg_ptr =  novatel_gps_msgs::InspvaxConstPtr(new novatel_gps_msgs::Inspvax(gps_msg));
  
  worker.inspvaxCallback(gps_msg_ptr);

  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::OPERATIONAL, pub->status.status);

  // Time passes
  ros::Time::setNow(startTime + ros::Duration(1.2));
  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::FAULT, pub->status.status);

  // Imu Message
  msg_ptr = sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(msg));
  
  worker.imuCallback(msg_ptr);

  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::OPERATIONAL, pub->status.status);

  // Time passes
  ros::Time::setNow(startTime + ros::Duration(1.7));
  worker.statusTimerCallback(event);
  ASSERT_EQ(cav_msgs::DriverStatus::FAULT, pub->status.status);
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleMock(&argc, argv);
    ros::init(argc, argv, "gps_nodelet_wrapper_worker_test", ros::init_options::AnonymousName);
    ros::Time::init();
    return RUN_ALL_TESTS();
}

