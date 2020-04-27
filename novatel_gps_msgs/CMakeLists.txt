cmake_minimum_required(VERSION 3.10)
project(novatel_gps_msgs)

set(CMAKE_CXX_STANDARD 14)

set(MSG_DEPS 
  std_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  msg/ClockSteering.msg
  msg/Gpgga.msg
  msg/Gpgsa.msg
  msg/Gpgsv.msg
  msg/Gphdt.msg
  msg/Gprmc.msg
  msg/Inscov.msg
  msg/Inspva.msg
  msg/Inspvax.msg
  msg/Insstdev.msg
  msg/NovatelCorrectedImuData.msg
  msg/NovatelDualAntennaHeading.msg
  msg/NovatelExtendedSolutionStatus.msg
  msg/NovatelHeading2.msg
  msg/NovatelMessageHeader.msg
  msg/NovatelPosition.msg
  msg/NovatelPsrdop2.msg
  msg/NovatelPsrdop2System.msg
  msg/NovatelReceiverStatus.msg
  msg/NovatelSignalMask.msg
  msg/NovatelUtmPosition.msg
  msg/NovatelVelocity.msg
  msg/NovatelXYZ.msg
  msg/Range.msg
  msg/RangeInformation.msg
  msg/Satellite.msg
  msg/Time.msg
  msg/Trackstat.msg
  msg/TrackstatChannel.msg
  srv/NovatelFRESET.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
