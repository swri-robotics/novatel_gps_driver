#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

sudo chmod -R +x /opt/carma/install
if [[ ! -z "$ROS1_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/ros/noetic/setup.bash
fi

cd ~/
if [[ ! -z "$ROS1_PACKAGES" ]]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS1_PACKAGES
else
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
