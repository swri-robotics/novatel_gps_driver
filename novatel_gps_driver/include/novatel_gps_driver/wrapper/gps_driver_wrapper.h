#pragma once
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

#include <string>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/DriverStatus.h>
#include <gps_common/GPSFix.h>

namespace novatel_gps_driver {
  /**
   * \class GPSDriverWrapper
   * \brief An interface which defines functions to publish gps data
   * 
   */
  class GPSDriverWrapper 
  {
    public:

      /**
       * \brief Virtual destructor to ensure delete safety for pointers to implementing classes
       * 
       */
      virtual ~GPSDriverWrapper() {};

      /**
       * \brief Publish a fused gps fix message
       * 
       * \param msg The message to publish
       * 
       */
      virtual void publishFixFused(const gps_common::GPSFix& msg) = 0;

      /**
       * \brief Publish a driver status message
       * 
       * \param msg The message to publish
       * 
       */
      virtual void publishStatus(const cav_msgs::DriverStatus& msg) = 0;

      /**
       * \brief Publish a system alert message
       * 
       * \param msg The message to publish
       * 
       */
      virtual void publishAlert(const cav_msgs::SystemAlert& msg) = 0;


      /**
       * \brief Returns the fully qualified name of the process
       * 
       * \return The process name
       * 
       */
      virtual std::string getQualifiedName() = 0;
  };
};