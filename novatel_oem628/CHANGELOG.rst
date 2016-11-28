^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_oem628
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
