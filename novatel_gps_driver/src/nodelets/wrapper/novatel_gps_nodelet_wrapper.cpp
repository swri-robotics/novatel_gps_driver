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

#include <nodelet/nodelet.h>
#include <exception>
#include <string>
#include <ros/ros.h>
#include <swri_roscpp/parameters.h>
#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>
#include <memory>
#include <cav_msgs/DriverStatus.h>
#include <boost/shared_ptr.hpp>
#include <novatel_gps_driver/wrapper/novatel_gps_nodelet_wrapper_worker.h>
#include <novatel_gps_driver/wrapper/gps_driver_wrapper.h>
#include <carma_utils/CARMANodeHandle.h>

namespace novatel_gps_driver
{
  // Helper class to serve as publishing interface for worker
  class APIPublisher : public GPSDriverWrapper {
    public:
      ros::Publisher fix_fused_pub_;
      ros::Publisher vel_raw_pub_;
      ros::Publisher heading_raw_pub_;
      ros::Publisher status_pub_;
      ros::Publisher alert_pub_;
      std::string name_;

      //// Overrides ////

      void publishFixFused(const gps_common::GPSFix& msg) override {
        fix_fused_pub_.publish(msg);
      }

      void publishStatus(const cav_msgs::DriverStatus& msg) override {
        status_pub_.publish(msg);
      }

      void publishAlert(const cav_msgs::SystemAlert& msg) override {
        alert_pub_.publish(msg);
      }

      std::string getQualifiedName() override {
        return name_;
      }

      void setName(const std::string& name) {
        name_ = name;
      }
  };

  class NovatelGpsNodeletWrapper : public nodelet::Nodelet
  {

    public:
      /**
       * Init method reads parameters and sets up publishers and subscribers.
       */
      void onInit()
      {
        ros::CARMANodeHandle nh(getNodeHandle());
        ros::CARMANodeHandle pnh(getPrivateNodeHandle());

        publisher_.reset(new APIPublisher());
        publisher_->setName(getName());

        publisher_->fix_fused_pub_ = nh.advertise<gps_common::GPSFix>("gnss_fix_fused", 10);
        publisher_->status_pub_ = nh.advertise<cav_msgs::DriverStatus>("driver_discovery", 10);
        publisher_->alert_pub_ = nh.advertise<cav_msgs::SystemAlert>("/system_alert", 10);


        WorkerConfig config;
        config.imu_timeout = ros::Duration(pnh.param("imu_timout", 0.5));
        config.gnss_timeout = ros::Duration(pnh.param("gnss_timout", 0.5));

        worker_.reset(new NovatelGPSNodeletWrapperWorker(config, publisher_));

        inspvax_sub_ = nh.subscribe("inspvax", 5, &NovatelGPSNodeletWrapperWorker::inspvaxCallback, worker_);
        imu_sub_ = nh.subscribe("imu_raw", 5, &NovatelGPSNodeletWrapperWorker::imuCallback, worker_);
        status_timer_ = nh.createTimer(ros::Duration(1.0), &NovatelGPSNodeletWrapperWorker::statusTimerCallback, worker_);

      }
    
    private: 
      boost::shared_ptr<NovatelGPSNodeletWrapperWorker> worker_;
      boost::shared_ptr<APIPublisher> publisher_;

      ros::Subscriber inspvax_sub_;
      ros::Subscriber imu_sub_;
      ros::Timer status_timer_;
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(novatel_gps_driver, NovatelGpsNodeletWrapper)
