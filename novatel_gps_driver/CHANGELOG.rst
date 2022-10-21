^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_oem628
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.1 (2022-10-21)
------------------
* Merge pull request `#116 <https://github.com/swri-robotics/novatel_gps_driver/issues/116>`_ from devrite/115-fix-humble-build-errors-tf2-and-rclcpp-components
  Fix humble build errors tf2 and rclcpp_components
* Include tf2 to geometry_msgs header
  Fixes `#115 <https://github.com/swri-robotics/novatel_gps_driver/issues/115>`_
* cmake: add rclcpp_components to deps
* Merge pull request `#109 <https://github.com/swri-robotics/novatel_gps_driver/issues/109>`_ from Smiffe/dashing-devel
  expected_rate based on polling_period parameter
* Merge pull request `#113 <https://github.com/swri-robotics/novatel_gps_driver/issues/113>`_ from ksuszka/galactic-launch-fix
  Fix example launch files to work under Galactic
* expected_rate based on polling_period parameter
* Merge pull request `#108 <https://github.com/swri-robotics/novatel_gps_driver/issues/108>`_ from ksuszka/dashing-devel
  Fix compilation warning on Galactic
* Contributors: David Anthony, Krzysztof Suszka, Markus Hofstaetter, PBed

4.1.0 (2020-10-06)
------------------
* Add param to disable invalid GPSFixes, replace GPRMC with BESTVEL (ROS2) (`#94 <https://github.com/swri-robotics/novatel_gps_driver/issues/94>`_)
* Use rclcpp::WallRate to sleep instead of boost (ROS2) (`#86 <https://github.com/swri-robotics/novatel_gps_driver/issues/86>`_)
* Refactor GPSFix generation (ROS2) (`#73 <https://github.com/swri-robotics/novatel_gps_driver/issues/73>`_)
* Contributors: P. J. Reed

4.0.3 (2020-03-06)
------------------
* Handle GPGSV log with 0 satellites correctly (`#77 <https://github.com/pjreed/novatel_gps_driver/issues/77>`_)
* Fill out error measurements in GPSFix messages (`#71 <https://github.com/pjreed/novatel_gps_driver/issues/71>`_)
* Fix covariance in GPSFix messages (`#69 <https://github.com/pjreed/novatel_gps_driver/issues/69>`_)
* Contributors: P. J. Reed

4.0.2 (2019-11-18)
------------------
* Add dependency on tf2_geometry_msgs (`#67 <https://github.com/swri-robotics/novatel_gps_driver/issues/67>`_)
* Contributors: P. J. Reed

4.0.1 (2019-11-15)
------------------
* Add missing dependencies (`#66 <https://github.com/swri-robotics/novatel_gps_driver/issues/66>`_)
* Add epson g370 (`#63 <https://github.com/swri-robotics/novatel_gps_driver/issues/63>`_)
* Contributors: P. J. Reed, icolwell-as

4.0.0 (2019-11-13)
------------------
* ROS2 Dashing support (`#61 <https://github.com/pjreed/novatel_gps_driver/issues/61>`_)
* Contributors: P. J. Reed

3.9.0 (2019-09-04)
------------------
* Use GPGGA quality to set GPSFix status (`#55 <https://github.com/swri-robotics/novatel_gps_driver/issues/55>`_)
* Publish INSPVAX logs (`#54 <https://github.com/swri-robotics/novatel_gps_driver/issues/54>`_)
* Add GPHDT message (`#51 <https://github.com/swri-robotics/novatel_gps_driver/issues/51>`_)
* Fix ascii header message id in novatel_gps.cpp (`#50 <https://github.com/swri-robotics/novatel_gps_driver/issues/50>`_)
* Correctly convert novatel header flags to bool and add tests for them (`#48 <https://github.com/swri-robotics/novatel_gps_driver/issues/48>`_)
* Add DUALANTENNAHEADING msg (`#46 <https://github.com/swri-robotics/novatel_gps_driver/issues/46>`_)
* Add HEADING2 msg (`#43 <https://github.com/swri-robotics/novatel_gps_driver/issues/43>`_)
* Add BESTXYZ msg (`#42 <https://github.com/swri-robotics/novatel_gps_driver/issues/42>`_)
* Contributors: Marcel Zeilinger, Matthew, Michael McConnell, P. J. Reed

3.8.0 (2019-06-04)
------------------
* Add fix for messages build order (`#41 <https://github.com/swri-robotics/novatel_gps_driver/issues/41>`_)
* Add clocksteering parsing (`#40 <https://github.com/swri-robotics/novatel_gps_driver/issues/40>`_)
* Only unlogall for the current port (`#36 <https://github.com/swri-robotics/novatel_gps_driver/issues/36>`_)
* Contributors: Matthew, P. J. Reed

3.7.0 (2019-03-22)
------------------
* Added IMU: Honeywell HG4930 AN01 (`#31 <https://github.com/swri-robotics/novatel_gps_driver/issues/31>`_)
* Added launch file to test an Ethernet interface. (`#33 <https://github.com/swri-robotics/novatel_gps_driver/issues/33>`_)
* Contributors: Rinda Gunjala, Zach Oakes

3.6.0 (2018-10-09)
------------------
* Allow setting the serial baud rate through serial_baud ROS parameter
* Add support for BESTUTM log
* Add support for INSPVAX log (#27)
* Contributors: Ellon Paiva Mendes, Sagnik Basu

3.5.0 (2018-07-17)
------------------
* Update documentation
* Fix parsing of gprmc messages for OEM4 models (#22)
* Finish serial commands with [CR][LF] (Carriage-Return Line-Feed) (#21)
* Adds configuration parameter span_frame_to_ros_frame. (#20)
* Adds additional IMUs defined in the OEM7 firmware. (#17)
* Enable the driver to determine IMU sample rate from the IMU type (#9)
* Contributors: Ellon Paiva Mendes, Joshua Whitley, Matthew, P. J. Reed

3.4.0 (2017-10-06)
------------------
* Fixes RPY orientation in IMU output (based on testing).
* Add service to novatel_gps_nodelet that issues a FRESET command on the indicated target, or STANDARD if none is provided
* Use find_library to look up the location of libpcap.so
* Contributors: Joshua Whitley, Matthew Bries, P. J. Reed

3.3.0 (2017-08-31)
------------------
* Fix crash for unexpected position type
* Add three-clause BSD license
* Publish sensor_msgs/Imu messages
* Use unlogall true
* Code cleanup
* Contributors: Edward Venator, P. J. Reed

3.2.0 (2017-07-21)
------------------
* Use size_t responsibly
* Remove duplicate message ID definition
* Reorganize parsing code
* Add binary message support
* Publish IMU data from NovAtel SPAN devices
* Contributors: Edward Venator, P. J. Reed

3.1.0 (2017-06-27)
------------------
* Add ethernet support
* Binary message support
* Contributors: P. J. Reed

3.0.1 (2017-05-08)
------------------
* Set UTC time correctly for message syncing
* Contributors: Edward Venator

3.0.0 (2017-04-03)
------------------
* Rename novatel_msgs to novatel_gps_msgs
* Fix catkin_lint warnings
* Add novatel_msgs as a dep to novatel_oem628
* Add support for Novatel Trackstat messages
* Remove debug error messages.
* Add gpgsv support to novatel driver nodelet.
* Add support for GPGSV messages in driver library.
* Add support for GPGSV message in novatel parser.
* Add support to ROS driver for Time message publishing
* Add support for accessing Time messages.
* Move messages into separate package.
* Update nodelet documentation.
* Add support for Novatel BESTVELA to ROS driver.
* Add support for BESTVELA to Novatel driver
* Add support for bestvel to message parser.
* Add new message for BESTVEL
* Add support for GPGSA messages.
* Add support to configure the Novatel GPS driver for any message type and frequency.
* Contributors: Edward Venator, P. J. Reed

2.9.0 (2017-01-11)
------------------
* Clean up wait_for_position member variable usage
* Merge repos on dismount and ivs; also fix some warnings
* Move GitLab CI config to correct location.
* Enable GitLab CI.
* Fix potential issue with how utc offset is applied.
* Change to always try to configure the device.
* Fix NMEA/novatel message synchronization issues
* Widening synchronization tolerance
* Switch to from \*_util to swri\_\*_util.
* Update example launch file for novatel nodelet.
* Contributors: Edward Venator, Jerry Towler, Kris Kozak, Marc Alban, P. J. Reed

2.8.0 (2017-01-05)
------------------
* Remove explicit serial device name check
* Add param to not publish sync diagnostic
* Contributors: Jason Gassaway, P. J. Reed

2.7.2 (2016-11-28)
------------------
* fixes issue caused by previous commit - no gps fix msgs being published
  previous commit caused no gps fix msgs to be published because msg buffers
  were small, and position msg arrived > 1 sec later than gpgga and gprmc msgs
  such that msgs were never synced. Added parameters to specify time sync
  tolerances and to optionally not wait for position msg. Position msg is not
  critical, it is only used to set position_covariance matrix.
* Contributors: Neal Seegmiller

2.7.1 (2016-08-04 05:18:06 -0500)
---------------------------------
* Reverts the syncing bug fix of previous commit
  This is a temporary fix that allows the novatel driver to output
  /localization/gps messages, even when BESTPOS messages are coming in with
  time stamps of ~1 seconds greater than GPGGA and GPRMC messages. With previous
  bug fix, these messages were simply popped off the buffer and no messages
  were published. Underlying issue should be fixed.
* Fix NMEA/novatel message synchronization issues
  - Switch parsing of string to doubles rather than floats to avoid precision
  errors.
  - Fix logic bug in synchronization loop.
* Updated novatel_oem628 so it uses swri_roscpp
* Working on migrating to Indigo.
* Install launch file.
* Contributors: Edward Venator, Jason Gassaway, Kris Kozak, Nicholas Alton, P. J. Reed

2.7.0 (2015-09-24 15:37:00 -0500)
---------------------------------
* Update example launch file for novatel nodelet.
* Fixing catkin_lint issues.
* Merge remote-tracking branch 'origin/catkin'
* Adds dependency so messages are generated before libraries.
* Add parameter to set GPS message frame_id.
  This commit adds a string parameter named 'frame_id' that will be
  copied into the header of every gps message.  This is intended to
  correspond to the GPS antenna coordinate frame so that other nodes can
  determine where the measurement was taken.  The frame_id defaults to
  empty to be compatible with previous behavior.
* Catkinizes novatel_oem628.
  Changes sync pulse time to std_msgs/Time to remove dependency on
  non-existing message in marti_sensor_msgs
* Add new NMEA message.
  The generic NMEA messages should be moved out of this repository in the future.
* Initial commit of novatel_oem628 package.
* Contributors: Edward Venator, Elliot Johnson, Kris Kozak, P. J. Reed
