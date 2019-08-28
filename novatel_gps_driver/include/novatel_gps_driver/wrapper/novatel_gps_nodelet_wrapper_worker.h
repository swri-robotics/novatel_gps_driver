
/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <cav_msgs/SystemAlert.h>
#include <novatel_gps_msgs/Inspvax.h>
#include <boost/shared_ptr.hpp>
#include <novatel_gps_driver/wrapper/worker_config.h>
#include <novatel_gps_driver/wrapper/gps_driver_wrapper.h>

/**
 * \class NovatelGPSNodeletWrapperWorker
 * \brief Algorithm logic for the NovatelGPSNodeletWrapper
 */
namespace novatel_gps_driver
{
  class NovatelGPSNodeletWrapperWorker
  {
    public:
      /**
       * \brief Constructor
       * \param config Configuration data for the worker algorithm
       * \param driver Object which can handle data transfer outside this wrapper
       */
      NovatelGPSNodeletWrapperWorker(WorkerConfig config, boost::shared_ptr<GPSDriverWrapper> publisher);
      /**
       * \brief Callback for INS data
       * \param msg The ins data message
       */ 
      void inspvaxCallback(const novatel_gps_msgs::InspvaxConstPtr& msg);
      /**
       * \brief Callback for imu data
       * \param msg The imu data message
       */ 
      void imuCallback(const sensor_msgs::ImuConstPtr& msg);
      /**
       * \brief Timer callback for sending driver status messages
       * \param event The timer event
       */ 
      void statusTimerCallback(const ros::TimerEvent& event);

    private: 
      WorkerConfig config_;
      boost::shared_ptr<GPSDriverWrapper> publisher_;
      cav_msgs::DriverStatus status_;
      ros::Time last_gnss_msg_;
      ros::Time last_imu_msg_;
  };

};