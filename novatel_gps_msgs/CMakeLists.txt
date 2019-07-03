cmake_minimum_required(VERSION 2.8.3)
project(novatel_gps_msgs)

set(MSG_DEPS 
  std_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_service_files(
  FILES
  NovatelFRESET.srv
)

add_message_files(DIRECTORY msg FILES
  ClockSteering.msg
  Gpgga.msg
  Gpgsa.msg
  Gpgsv.msg
  Gprmc.msg
  Inscov.msg
  Inspva.msg
  Inspvax.msg
  Insstdev.msg
  NovatelCorrectedImuData.msg
  NovatelExtendedSolutionStatus.msg
  NovatelHeading2.msg
  NovatelMessageHeader.msg
  NovatelPosition.msg
  NovatelUtmPosition.msg
  NovatelReceiverStatus.msg
  NovatelSignalMask.msg
  NovatelVelocity.msg
  NovatelXYZ.msg
  RangeInformation.msg
  Range.msg
  Satellite.msg
  Time.msg
  TrackstatChannel.msg
  Trackstat.msg
)
generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)
