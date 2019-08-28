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
        publisher_->alert_pub_ = nh.advertise<cav_msgs::SystemAlert>("system_alert", 10);


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
