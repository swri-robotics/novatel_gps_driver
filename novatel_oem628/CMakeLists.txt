cmake_minimum_required(VERSION 2.8.3)

project(novatel_oem628)

set(DEPS
  diagnostic_msgs 
  diagnostic_updater 
  gps_common 
  nav_msgs 
  nodelet 
  novatel_gps_msgs
  roscpp
  sensor_msgs
  std_msgs
  swri_math_util 
  swri_roscpp
  swri_serial_util
  swri_string_util 
  tf
)

find_package(catkin REQUIRED COMPONENTS ${DEPS})

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS ${DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelets
  DEPENDS Boost
)

add_library(${PROJECT_NAME} 
  src/novatel_gps.cpp
  src/novatel_message_parser.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### Nodelets ###
add_library(${PROJECT_NAME}_nodelets 
  src/nodelets/novatel_gps_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelets
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

### Nodes ###
add_executable(novatel_gps_node
  src/nodes/novatel_gps_node.cpp
)
target_link_libraries(novatel_gps_node 
  ${catkin_LIBRARIES}
)

### Install Libraries and Headers ###
install(TARGETS 
    novatel_gps_node
    ${PROJECT_NAME} 
    ${PROJECT_NAME}_nodelets
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
