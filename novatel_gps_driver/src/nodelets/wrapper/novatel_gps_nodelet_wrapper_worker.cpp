/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <novatel_gps_driver/wrapper/novatel_gps_nodelet_wrapper_worker.h>
#include <tuple>
#include <uncertainty_tools/uncertainty_tools.h>

namespace novatel_gps_driver
{

  NovatelGPSNodeletWrapperWorker::NovatelGPSNodeletWrapperWorker(WorkerConfig config, boost::shared_ptr<GPSDriverWrapper> publisher) : 
    config_(config), publisher_(publisher)
  {
    status_.imu = true;
    status_.gnss = true;
    status_.name = publisher_->getQualifiedName();
  }

  void NovatelGPSNodeletWrapperWorker::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    last_imu_msg_ = ros::Time::now();
  }

  void NovatelGPSNodeletWrapperWorker::inspvaxCallback(const novatel_gps_msgs::InspvaxConstPtr& msg) {

    last_gnss_msg_ = ros::Time::now();

    gps_common::GPSFix fix_msg;
    // NOTE: At the moment CARMA's hardware interfaces do not use GPS status information
    // Therefore that information is not currently populated in this message
    // Convert position
    fix_msg.header = msg->header;
    fix_msg.latitude = msg->latitude;
    fix_msg.longitude = msg->longitude;
    fix_msg.altitude = msg->altitude; 
    // Set position covariance
    fix_msg.position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_msg.position_covariance[0] = msg->longitude_std * msg->longitude_std;
    fix_msg.position_covariance[4] = msg->latitude_std * msg->latitude_std;
    fix_msg.position_covariance[8] = msg->altitude_std * msg->altitude_std;
    
    // Convert orientation
    // NOTE: It is unclear in the message spec what dip represents so it will be ignored
    fix_msg.track = msg->azimuth;
    fix_msg.pitch = msg->pitch;
    fix_msg.roll = msg->roll;

    // GPSFix messages request uncertainty reported with 95% confidence interval. That is 2 times the standard deviation
    fix_msg.err_track = 2.0 * msg->azimuth_std;
    fix_msg.err_pitch = 2.0 * msg->pitch_std;
    fix_msg.err_roll = 2.0 * msg->roll_std;

    // Convert Velocity
    std::vector<double> components, variances;
    components.push_back(msg->north_velocity);
    variances.push_back(2.0 * msg->north_velocity_std);

    components.push_back(msg->east_velocity);
    variances.push_back(2.0 * msg->east_velocity_std);

    std::tuple<double,double> speed_tuple = uncertainty_tools::computeVectorMagnitudeAndUncertainty(components, variances);
    fix_msg.speed = std::get<0>(speed_tuple);
    fix_msg.climb = msg->up_velocity;

    fix_msg.err_climb = 2.0 * msg->up_velocity_std;
    fix_msg.err_speed = std::get<1>(speed_tuple);

    publisher_->publishFixFused(fix_msg);
  }

  void NovatelGPSNodeletWrapperWorker::statusTimerCallback(const ros::TimerEvent& event) {
    ros::Time now = ros::Time::now();
    switch(status_.status) {
      case cav_msgs::DriverStatus::OFF:
        if (last_gnss_msg_ != ros::Time(0) && last_imu_msg_ != ros::Time(0)) {
          status_.status = cav_msgs::DriverStatus::OPERATIONAL;
        }
        break;
      case cav_msgs::DriverStatus::OPERATIONAL:
        if (now - last_gnss_msg_ > config_.gnss_timeout || now - last_imu_msg_ > config_.imu_timeout) {
          status_.status = cav_msgs::DriverStatus::FAULT;
        }
        break;
      case cav_msgs::DriverStatus::DEGRADED:
        if (now - last_gnss_msg_ > config_.gnss_timeout || now - last_imu_msg_ > config_.imu_timeout) {
          status_.status = cav_msgs::DriverStatus::FAULT;
        }
        // TODO add logic for checking satellite coverage and use that for degraded status
        break;
      case cav_msgs::DriverStatus::FAULT: 
        if (now - last_gnss_msg_ < config_.gnss_timeout && now - last_imu_msg_ < config_.imu_timeout) {
          status_.status = cav_msgs::DriverStatus::OPERATIONAL;
        }
        break;
      default:
        throw std::invalid_argument("Unknown stauts type: " + std::to_string(status_.status));
    }

    publisher_->publishStatus(status_);
  }
};