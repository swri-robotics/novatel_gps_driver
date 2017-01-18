cmake_minimum_required(VERSION 2.8.3)

project(novatel_oem628)

set(MSGS_DEPS 
  std_msgs)

set(BUILD_DEPS
  ${MSGS_DEPS} 
  diagnostic_msgs 
  diagnostic_updater 
  gps_common 
  swri_math_util 
  message_generation 
  nav_msgs 
  nodelet 
  roscpp 
  swri_serial_util 
  swri_string_util 
  swri_roscpp
  tf)

set(RUNTIME_DEPS
  ${MSGS_DEPS} 
  diagnostic_msgs 
  diagnostic_updater 
  gps_common 
  swri_math_util 
  message_runtime 
  nav_msgs 
  nodelet 
  roscpp 
  swri_serial_util 
  swri_string_util 
  swri_roscpp
  tf)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})

find_package(Boost REQUIRED COMPONENTS system)

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
)

generate_messages(DEPENDENCIES ${MSGS_DEPS})

include_directories(include
  ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelets)

add_library(${PROJECT_NAME} 
  src/novatel_gps.cpp
	src/novatel_message_parser.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
# Add dependency so that library does not build until messages are built
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})


### Nodelets ###
add_library(${PROJECT_NAME}_nodelets src/nodelets/novatel_gps_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})
# Add dependency so that library does not build until messages are built
add_dependencies(${PROJECT_NAME}_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS})

### Nodes ###
add_executable(novatel_gps_node src/nodes/novatel_gps_node.cpp)
target_link_libraries(novatel_gps_node ${catkin_LIBRARIES})

### Install Libraries and Headers ###
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelets novatel_gps_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

### Install plugin XML ###
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

### Install Launch Files ###
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
