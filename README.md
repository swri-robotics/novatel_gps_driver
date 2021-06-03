NovAtel GPS Driver
![CI](https://github.com/swri-robotics/novatel_gps_driver/workflows/CI/badge.svg)
==================

Overview
--------

This is a C++ [ROS](http://www.ros.org/) driver for [NovAtel](https://www.novatel.com/) GPS / GNSS Receivers.  

Features:

- Fast and robust
- Supports serial, TCP, and UDP connections
- Can play back PCAP capture logs to test behavior
- Supports a variety of common NovAtel logs
- Easy to add support for more log types
- Supports ASCII and binary-format NovAtel logs
- Can synchronize `BESTPOS`, `BESTVEL`, and `PSRDOP2` logs together in order to produce 
[gps_common/GPSFix](http://docs.ros.org/kinetic/api/gps_common/html/msg/GPSFix.html) messages
- Tested with OEM4, OEM6, and OEM7 receivers
- Can produce IMU data from receives with SPAN support

It has been tested primarily on NovAtel OEM628 receivers, but it has been used with
various OEM4, OEM6, and OEM7 devices. Please let 
[the maintainers](mailto:preed@swri.org,kkozak@swri.org) know of your success
or failure in using this with other devices so we can update this page appropriately.

Usage
-----

The `master` branch for this driver functions on ROS Kinetic and Melodic;
`dashing-devel` supports ROS 2 Dashing.  To install the binary packages, first install ROS,
then just run:

```bash
sudo apt-get install ros-${ROS_DISTRO}-novatel-gps-driver
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
      publish_novatel_velocity: true
      publish_novatel_positions: true
      publish_novatel_psrdop2: true
    </rosparam>
  </node>
</launch>
```

`gps_common/GPSFix` messages will always be published, but by default, other message 
types are not.  See the config parameters for a list of which other message types can be
enabled.

Packages
--------
1. ### `novatel_gps_msgs`

    ROS messages that represent NovAtel message logs.  Each supported log
    should have its own message type.  A list of official log types can be found
    at https://docs.novatel.com/OEM7/Content/Commands/CommandsLogs.htm .
2. ### `novatel_gps_driver`

    A C++ library with an accompanying ROS nodelet and node that can connect to 
    a NovAtel device over a serial, TCP, or UDP connection and translate NovAtel
    logs into ROS messages.

Nodelets
--------

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
        - `publish_gphdt`: `true` to publish novatel_gps_msgs/Gphdt messages.
            - Default: `false`
        - `publish_imu_messages`: `true` to publish novatel_gps_msgs/NovatelCorrectedImuData, novatel_gps_msgs/Inspva,
        novatel_gps_msgs/Inspvax, novatel_gps_msgs/Insstdev, and sensor_msgs/Imu messages.
            - Default: `false`
        - `publish_nmea_messages`: `true` to publish novatel_gps_msgs/Gpgga and novatel_gps_msgs/Gprmc messages.
            - Default: `false`
        - `publish_novatel_dual_antenna_heading`: `true` to publish novatel_gps_msgs/NovatelDualAntennaHeading messages.
            - Default: `false`
        - `publish_novatel_heading2`: `true` to publish novatel_gps_msgs/Heading2 messages.
            - Default: `false`
        - `publish_novatel_positions`: `true` to publish novatel_gps_msgs/NovatelPosition messages.  Note that even if
        this is false, these logs will always be requested from the receiver in order to generate `gps_msgs/GPSFix`
        messages.
            - Default: `false`
        - `publish_novatel_psrdop2`: `true` to publish novatel_gps_msgs/NovatelPsrdop2 messages.  If set, the data from
        these messages will be used to fill in the DoP values in `gps_msgs/GPSFix` messages.  Note that these messages
        are only published when the values change, not at the standard polling rate.
            - Default: `false`
        - `publish_novatel_utm_positions`: `true` to publish novatel_gps_msgs/NovatelUtmPosition messages.
            - Default: `false`
        - `publish_novatel_velocity`: `true` to publish novatel_gps_msgs/NovatelVelocity messages.  If set, the data
        from these messages will be used to fill in the speed and track values in `gps_msgs/GPSFix` messages.
            - Default: `false`
        - `publish_novatel_xyz_positions`: `true` to publish novatel_gps_msgs/NovatelXYZ messages.
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
        - `wait_for_sync`: `true` in order to wait for both BESTPOS and BESTVEL messages to arrive before publishing
        `gps_msgs/GPSFix` messages.  If this is `false`, GPSFix messages will be published immediately when BESTPOS
        messages are received, but a side effect is that the driver will often be unable to fill in the speed & track
        fields.  This has no effect if `publish_novatel_velocity` is `false`.
            - Default: `true`
    2. **ROS Topic Subscriptions**
        - `/gps_sync` *(std_msgs/Time)*: *(optional)* Timestamped sync pulses from a DIO module. 
    These are used to improve the accuracy of the time stamps of the messages published.
    3. **ROS Topic Publications**
        - `/bestpos` *(novatel_gps_msgs/NovatelPosition)*: [BESTPOS](http://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm) logs
        - `/bestutm` *(novatel_gps_msgs/NovatelUtmPosition)*: [BESTUTM](http://docs.novatel.com/OEM7/Content/Logs/BESTUTM.htm) logs
        - `/bestvel` *(novatel_gps_msgs/NovatelVelocity)*: [BESTVEL](http://docs.novatel.com/OEM7/Content/Logs/BESTVEL.htm) logs
        - `/bestxyz` *(novatel_gps_msgs/NovatelXYZ)*: [BESTXYZ](http://docs.novatel.com/OEM7/Content/Logs/BESTXYZ.htm) logs
        - `/clocksteering` *(novatel_gps_msgs/ClockSteering)*: [CLOCKSTEERING](http://docs.novatel.com/OEM7/Content/Logs/CLOCKSTEERING.htm) logs
        - `/corrimudata` *(novatel_gps_msgs/NovatelCorrectedImuData)*: [CORRIMUDATA](http://docs.novatel.com/OEM7/Content/SPAN_Logs/CORRIMUDATA.htm) logs
        - `/diagnostics` *(diagnostic_msgs/DiagnosticArray)*: ROS diagnostics
        - `/dual_antenna_heading` *(novatel_gps_msgs/NovatelDualAntennaHeading)*: [DUALANTENNAHEADING](http://docs.novatel.com/OEM7/Content/Logs/DUALANTENNAHEADING.htm) logs
        - `/fix` *([sensor_msgs/NavSatFix](https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/NavSatFix.html))*: GPSFix messages converted to NavSatFix messages
        - `/gpgga` *(novatel_gps_msgs/Gpgga)*: [GPGGA](http://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm) logs
        - `/gpgsa` *(novatel_gps_msgs/Gpgsa)*: [GPGSA](http://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm) logs
        - `/gpgsv` *(novatel_gps_msgs/Gpgsv)*: [GPGSV](http://docs.novatel.com/OEM7/Content/Logs/GPGSV.htm) logs
        - `/gprmc` *(novatel_gps_msgs/Gprmc)*: [GPRMC](http://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm) logs
        - `/gps` *([gps_msgs/GPSFix](http://docs.ros.org/kinetic/api/gps_common/html/msg/GPSFix.html))*: Fixes produced by combining BESTVEL, PSRDOP2 and BESTPOS messages together
            - **Note**:  GPSFix messages will always be published regardless of what other types are enabled.        
        - `/heading2` *(novatel_gps_msgs/NovatelHeadin2)*: [HEADING2](http://docs.novatel.com/OEM7/Content/Logs/HEADING2.htm) logs
        - `/imu` *([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))*: CORRIMUDATA logs converted to Imu messages
        - `/inspva` *(novatel_gps_msgs/Inspva)*: [INSPVA](http://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm) logs
        - `/inspvax` *(novatel_gps_msgs/Inspvax)*: [INSPVAX](http://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVAX.htm) logs
        - `/insstdev` *(novatel_gps_msgs/Insstdev)*: [INSSTDEV](http://docs.novatel.com/OEM7/Content/SPAN_Logs/INSSTDEV.htm) logs
        - `/psrdop2` *(novatel_gps_msgs/Psrdop2)*: [PSRDOP2](https://docs.novatel.com/OEM7/Content/Logs/PSRDOP2.htm) logs
        - `/range` *(novatel_gps_msgs/Range)*: [RANGE](http://docs.novatel.com/OEM7/Content/Logs/RANGE.htm) logs
        - `/rosout` *(rosgraph_msgs/Log)*: Console output
        - `/time` *(novatel_gps_msgs/Time)*: [TIME](http://docs.novatel.com/OEM7/Content/Logs/TIME.htm) logs
        - `/trackstat` *(novatel_gps_msgs/Trackstat)*: [TRACKSTAT](http://docs.novatel.com/OEM7/Content/Logs/TRACKSTAT.htm) logs

Adding New Logs
---------------

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
