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
  msg/Trackstat.msg
  msg/TrackstatChannel.msg
  msg/NovatelRxStatus.msg
  srv/NovatelFRESET.srv
)

# The TIME log's message is called NovatelTime on Rolling and newer and Time on
# Lyrical and older.
#
# rosidl 5.3.0 started emitting an unnamespaced `_Time__ros_msg_type` alias into
# the FastRTPS typesupport of every message named Time. builtin_interfaces/Time
# emits the same alias, and since our message embeds a std_msgs/Header the two
# always land in one translation unit, so the package no longer compiles with a
# message named Time. Renaming it is the fix, but the rename is an API break, so
# only apply it where the old name genuinely cannot be built.
#
# Gate on rosidl rather than ROS_DISTRO so source builds and future distros sort
# themselves out: Rolling is 5.3.x, Lyrical 5.2.x, Kilted/Jazzy/Humble older.
if(NOT rosidl_cmake_VERSION)
  message(FATAL_ERROR
    "rosidl_cmake_VERSION is not set; cannot determine whether to build "
    "msg/NovatelTime.msg or msg/Time.msg.")
endif()

if(rosidl_cmake_VERSION VERSION_GREATER_EQUAL "5.3.0")
  list(APPEND MSG_FILES msg/NovatelTime.msg)
else()
  # Same definition under the historical name. Copied into the build tree rather
  # than duplicated in the source tree so the two cannot drift apart, and passed
  # as the `<absolute base>:<relative path>` tuple that rosidl_generate_interfaces
  # requires for interfaces outside CMAKE_CURRENT_SOURCE_DIR.
  configure_file(msg/NovatelTime.msg "${CMAKE_CURRENT_BINARY_DIR}/msg/Time.msg" COPYONLY)
  list(APPEND MSG_FILES "${CMAKE_CURRENT_BINARY_DIR}:msg/Time.msg")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
