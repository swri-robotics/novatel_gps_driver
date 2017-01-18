cmake_minimum_required(VERSION 2.8.3)
project(novatel_msgs)

set(MSG_DEPS 
  std_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_message_files(DIRECTORY msg FILES
  Gpgga.msg
  Gpgsa.msg
  Gprmc.msg
  NovatelExtendedSolutionStatus.msg
  NovatelMessageHeader.msg
  NovatelPosition.msg
  NovatelReceiverStatus.msg
  NovatelSignalMask.msg
  NovatelVelocity.msg
  Time.msg
)
generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime
)

