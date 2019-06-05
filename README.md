NovAtel GPS Driver [![Build Status](https://travis-ci.org/swri-robotics/novatel_gps_driver.svg?branch=master)](https://travis-ci.org/swri-robotics/novatel_gps_driver)
==================

1. Overview
-----------

This is a C++ [ROS](http://www.ros.org/) driver for [NovAtel](https://www.novatel.com/) GPS / GNSS Receivers.  

Features:

- Fast and robust
- Supports serial, TCP, and UDP connections
- Can play back PCAP capture logs to test behavior
- Supports a variety of common NovAtel logs
- Easy to add support for more log types
- Supports ASCII and binary-format NovAtel logs
- Can synchronize `GPGGA`, `GPRMC`, and `BESTPOS` logs together in order to produce 
[gps_common/GPSFix](http://docs.ros.org/kinetic/api/gps_common/html/msg/GPSFix.html) messages
- Compatible with OEM4, OEM6, and OEM7 receivers
- Can produce IMU data from receives with SPAN support

It has been tested primarily on NovAtel OEM628 receivers, but it has been used with
various OEM4, OEM6, and OEM7 devices. Please let 
[the maintainers](mailto:preed@swri.org,kkozak@swri.org) know of your success
or failure in using this with other devices so we can update this page appropriately.

2. Usage
--------

The driver should function on ROS Kinetic and Melodic, and binary packages are available
for both of them.  To install them, first install ROS, then just run:

```bash
sudo apt-get install ros-${ROSDISTRO}-novatel-gps-driver
```

If you'd like to build it from source using [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html):

```bash
mkdir -p novatel/src
cd novatel
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
cd src
git clone https://github.com/swri-robotics/novatel_gps_driver
rosdep install . --from-paths -i
catkin build
```

Then create a `.launch` file and configure it as desired:

```xml
<?xml version="1.0"?>
<launch>
  <node name="novatel"
        pkg="nodelet" type="nodelet"
        args="standalone novatel_gps_driver/novatel_gps_nodelet">
    <rosparam>
      connection_type: serial
      device: /dev/ttyUSB0
      frame_id: /gps
    </rosparam>
  </node>
</launch>
```

`gps_common/GPSFix` messages will always be published, but by default, other message 
types are not.  See the config parameters for a list of which other message types can be
enabled.

3. Packages
-----------
1. ### `novatel_gps_msgs`

    ROS messages that represent NovAtel message logs.  Each supported log
    should have its own message type.  A list of official log types can be found
    at http://docs.novatel.com/OEM7/Content/Logs/Log_Reference.htm .
2. ### `novatel_gps_driver`

    A C++ library with an accompanying ROS nodelet and node that can connect to 
    a NovAtel device over a serial, TCP, or UDP connection and translate NovAtel
    logs into ROS messages.

4. Nodelets
-----------

1. ### `novatel_gps_driver/novatel_gps_nodelet`
    1. **ROS Parameters**
        - `connection_type`: Type of physical connection to the device
            - One of `serial`, `tcp`, `udp`, or `pcap`
            - Default: `serial`
        - `device`: Location of device connection.
            - For `serial` connections, the device node; e. g., `/dev/ttyUSB0`
            - For `tcp` or `udp` connections, a `host:port` specification.
                - If the host is omitted, it will listen for connections on the specified port.
                - If the port is omitted, `3001` will be used as the default for TCP connections
                and `3002` as the default for UDP connections.
            - For `pcap` connections, the location of a `.pcap` capture file. 
            Note that the node will exit automatically after finishing playback.
            - Default: Empty
        - `frame_id`: ROS TF frame to place in the header of published messages.
            - Default: Empty
        - `gpgga_gprmc_sync_tol`: Tolerance (in seconds) for synchronizing GPGGA and GPRMC logs.
            - Default: `0.01`
        - `gpgga_position_sync_tol`: Tolerance (in seconds) for synchronizing GPGGA and BESTPOS logs.
            - Default `0.01`
        - `imu_frame_id`: TF frame id to use in IMU messages.
            - Default: Empty
        - `imu_rate`: Desired rate in Hz for IMU messages.
            - This is set as the rate for `CORRIMUDATA` logs.
            - Default: `100`
        - `polling_period`: Desired period between GPS messages. 
            - This will be set as the period for `GPGGA`, `GPRMC`, `GPGSA`, `BESTPOS`, 
            and `BESTVEL` logs.
            - Default: `0.05` (20 Hz)
        - `publish_clocksteering`: `true` to publish novatel_gps_msgs/ClockSteering messages.
            - Default: `false`
        - `publish_diagnostics`: `true` to publish node diagnostics.
            - Default: `false`
        - `publish_gpgsa`: `true` to publish novatel_gps_msgs/Gpgsa messages.
            - Default: `false`
        - `publish_gpgsv`: `true` to publish novatel_gps_msgs/Gpgsv messages.
            - Default: `false`
        - `publish_imu_messages`: `true` to publish novatel_gps_msgs/NovatelCorrectedImuData, novatel_gps_msgs/Inspva,
         novatel_gps_msgs/Insstdev, and sensor_msgs/Imu messages.
            - Default: `false`
        - `publish_nmea_messages`: `true` to publish novatel_gps_msgs/Gpgga and novatel_gps_msgs/Gprmc messages.
            - Default: `false`
        - `publish_novatel_positions`: `true` to publish novatel_gps_msgs/NovatelPosition messages.
            - Default: `false`
        - `publish_novatel_utm_positions`: `true` to publish novatel_gps_msgs/NovatelUtmPosition messages.
            - Default: `false`
        - `publish_novatel_velocity`: `true` to publish novatel_gps_msgs/NovatelVelocity messages.
            - Default: `false`
        - `publish_range_messages`: `true` to publish novatel_gps_msgs/Range messages.
            - Default: `false`
        - `publish_sync_diagnostic`: If true, publish a time Sync diagnostic.
            - Ignored if `publish_diagnostics` is false.
            - Default: `true`
        - `publish_time_messages`: `true` to publish novatel_gps_msgs/Time messages.
            - Default: `false`
        - `publish_trackstat`: `true` to publish novatel_gps_msgs/Trackstat messages.
            - Default: `false`
        - `reconnect_delay_s`: Second delay between reconnection attempts.
            - Default: `0.5`
        - `serial_baud`: Select the serial baud rate to be used in a serial connection.
            - Default: `115200`
        - `use_binary_messages`: `true` to request binary NovAtel logs, `false` to request ASCII.
            - Binary logs are much more efficient and effectively required for IMU data,
            but ASCII logs are easier to parse for a human.
            - Default: `false`
        - `wait_for_position`: `true` in order to wait for BESTPOS logs before publishing GPSFix messages.
            - Default: `false`
    2. **ROS Topic Subscriptions**
        - `/gps_sync` *(std_msgs/Time)*: *(optional)* Timestamped sync pulses from a DIO module. 
    These are used to improve the accuracy of the time stamps of the messages published.
    3. **ROS Topic Publications**
        - `/bestpos` *(novatel_gps_msgs/NovatelPosition)*: [BESTPOS](http://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm) logs
        - `/bestutm` *(novatel_gps_msgs/NovatelUtmPosition)*: [BESTUTM](http://docs.novatel.com/OEM7/Content/Logs/BESTUTM.htm) logs
        - `/bestvel` *(novatel_gps_msgs/NovatelVelocity)*: [BESTVEL](http://docs.novatel.com/OEM7/Content/Logs/BESTVEL.htm) logs
        - `/clocksteering` *(novatel_gps_msgs/ClockSteering)*: [CLOCKSTEERING](http://docs.novatel.com/OEM7/Content/Logs/CLOCKSTEERING.htm) logs
        - `/corrimudata` *(novatel_gps_msgs/NovatelCorrectedImuData)*: [CORRIMUDATA](http://docs.novatel.com/OEM7/Content/SPAN_Logs/CORRIMUDATA.htm) logs
        - `/diagnostics` *(diagnostic_msgs/DiagnosticArray)*: ROS diagnostics
        - `/gpgga` *(novatel_gps_msgs/Gpgga)*: [GPGGA](http://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm) logs
        - `/gpgsa` *(novatel_gps_msgs/Gpgsa)*: [GPGSA](http://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm) logs
        - `/gpgsv` *(novatel_gps_msgs/Gpgsv)*: [GPGSV](http://docs.novatel.com/OEM7/Content/Logs/GPGSV.htm) logs
        - `/gprmc` *(novatel_gps_msgs/Gprmc)*: [GPRMC](http://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm) logs
        - `/gps` *([gps_common/GPSFix](http://docs.ros.org/kinetic/api/gps_common/html/msg/GPSFix.html))*: Fixes produced by combining GPGGA, GPRMC, and BESTPOS messages together
            - **Note**:  GPSFix messages will always be published regardless of what other types are enabled.
        - `/imu` *([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))*: CORRIMUDATA logs converted to Imu messages
        - `/range` *(novatel_gps_msgs/Range)*: [RANGE](http://docs.novatel.com/OEM7/Content/Logs/RANGE.htm) logs
        - `/rosout` *(rosgraph_msgs/Log)*: Console output
        - `/time` *(novatel_gps_msgs/Time)*: [TIME](http://docs.novatel.com/OEM7/Content/Logs/TIME.htm) logs
        - `/trackstat` *(novatel_gps_msgs/Trackstat)*: [TRACKSTAT](http://docs.novatel.com/OEM7/Content/Logs/TRACKSTAT.htm) logs

5. Adding New Logs
------------------

Do you need support for a new log type?  Follow these steps:

1. Find the log reference in the [official documentation](http://docs.novatel.com/OEM7/Content/Logs/Log_Reference.htm).
2. Add a new .msg file to the `novatel_gps_msgs` package.
3. Add a new class in the `novatel_gps_driver` package that extends the `novatel_gps_driver::MessageParser` class that
can parse the log and return the appropriate ROS message.
4. Modify the `novatel_gps_driver::NovatelGps` class:
    1. Add an instance of your parser, a buffer for storing parsed messages, and a method for retrieving them.
    2. Modify the `NovatelGps::ParseBinaryMessage`, `NovatelGps::ParseNovatelSentence`, 
    or `NovatelGps::ParseNmeaSentence` methods to use your parser to parse the new message type
    and store it in the appropriate buffer.
5. Modify the `novatel_gps_driver::NovatelGpsNodelet` class:
    1. Add a configuration parameter to enable the new message type.
    2. Add a publisher for publishing it.
    3. Modify `NovatelGpsNodelet::CheckDeviceForData` to retrieve messages from
    the appropriate buffer and publish them.
6. Add a new unit test to `novatel_gps_driver/tests/parser_tests.cpp` to test your parser.
