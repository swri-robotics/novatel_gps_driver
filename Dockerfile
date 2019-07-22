#  Copyright (C) 2018-2019 LEIDOS.
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

FROM usdotfhwastol/carma-base:2.8.3 as setup

RUN sudo apt-get update \
    && sudo apt-get install -y libpcap-dev ros-kinetic-gps-common ros-kinetic-swri-math-util ros-kinetic-swri-roscpp ros-kinetic-swri-serial-util ros-kinetic-swri-string-util ros-kinetic-swri-nodelet

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN chmod +x ~/src/docker/checkout.sh && chown carma ~/src/docker/checkout.sh 
RUN chmod +x ~/src/docker/install.sh && chown carma ~/src/docker/install.sh 
RUN ~/src/docker/checkout.sh
RUN ~/src/docker/install.sh

FROM usdotfhwastol/carma-base:2.8.3

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-novatel-gps-driver"
LABEL org.label-schema.description="carma novatel gps driver for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/CARMANovatelGpsDriver/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=setup /home/carma/install /opt/carma/app/bin
RUN sudo chmod -R +x /opt/carma/app/bin

CMD [ "wait-for-it.sh", "localhost:11311", "--", "roslaunch", "novatel_gps_driver", "novatel_gps_driver_eth.launch", "remap_ns:=/saxton_cav/drivers" ]