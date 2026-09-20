cmake_minimum_required(VERSION 3.14.4)

set(CMAKE_CXX_STANDARD 17)

project(novatel_gps_driver)

# System dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED CONFIG)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(novatel_gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_serial_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# libpcap; it doesn't provide PkgConfig or CMake modules, so wrap what find_library
# returns in an imported target to keep the link lists below made of targets only.
find_library(libpcap_LIBRARIES pcap)
if(NOT libpcap_LIBRARIES)
  message(FATAL_ERROR "Could not find libpcap.  Install it (libpcap-dev on Debian and Ubuntu).")
endif()
if(NOT TARGET pcap::pcap)
  add_library(pcap::pcap UNKNOWN IMPORTED)
  set_target_properties(pcap::pcap PROPERTIES
    IMPORTED_LOCATION "${libpcap_LIBRARIES}")
endif()

### Driver library ###
add_library(${PROJECT_NAME}
  src/novatel_gps.cpp
  src/novatel_message_extractor.cpp
  src/parsers/bestpos.cpp
  src/parsers/bestutm.cpp
  src/parsers/bestvel.cpp
  src/parsers/bestxyz.cpp
  src/parsers/clocksteering.cpp
  src/parsers/corrimudata.cpp
  src/parsers/corrimus.cpp
  src/parsers/dual_antenna_heading.cpp
  src/parsers/gpgga.cpp
  src/parsers/gpgsa.cpp
  src/parsers/gpgsv.cpp
  src/parsers/gphdt.cpp
  src/parsers/gprmc.cpp
  src/parsers/header.cpp
  src/parsers/heading2.cpp
  src/parsers/inscov.cpp
  src/parsers/inspva.cpp
  src/parsers/inspvas.cpp
  src/parsers/inspvax.cpp
  src/parsers/insstdev.cpp
  src/parsers/insupdatestatus.cpp
  src/parsers/parsing_utils.cpp
  src/parsers/psrdop2.cpp
  src/parsers/range.cpp
  src/parsers/rawdmi.cpp
  src/parsers/rawimux.cpp
  src/parsers/time.cpp
  src/parsers/trackstat.cpp
  src/parsers/rxstatus.cpp
)
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
# PUBLIC for anything the installed headers include, PRIVATE for anything only the
# sources use, so consumers are told what they actually need.
#
# Plain libraries are linked through their namespaced target.  Interface packages
# are linked through ${<package>_TARGETS}, the list of typesupport targets, because
# the single ${<package>}::${<package>} aggregate that would replace it is not
# generated on Humble.
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Boost::headers
    diagnostic_updater::diagnostic_updater
    rclcpp::rclcpp
    swri_serial_util::swri_serial_util
    ${diagnostic_msgs_TARGETS}
    ${gps_msgs_TARGETS}
    ${nmea_msgs_TARGETS}
    ${novatel_gps_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
  PRIVATE
    pcap::pcap
    swri_math_util::swri_math_util
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)

### ROS Node ###
add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/novatel_gps_node.cpp
)
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_node(${PROJECT_NAME}_nodes 
  PLUGIN "novatel_gps_driver::NovatelGpsNode"
  EXECUTABLE "novatel_gps_node"
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(${PROJECT_NAME}_nodes
  PUBLIC
    ${PROJECT_NAME}
    ${rtcm_msgs_TARGETS}
  PRIVATE
    rclcpp_components::component
    swri_math_util::swri_math_util
)

### Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(parser_tests test/parser_tests.cpp)
  target_link_libraries(parser_tests
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
  )

  ament_add_gtest(novatel_gps_tests test/novatel_gps_tests.cpp)
  target_link_libraries(novatel_gps_tests
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
  )

  install(DIRECTORY test/
    DESTINATION test
    FILES_MATCHING PATTERN "*.pcap"
  )
endif()

### Install Libraries and Headers ###
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Headers go under their own package directory so merged installs can't collide.
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

### Install Launch Files ###
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# The export set carries the include directories and link interface, so downstream
# packages get novatel_gps_driver::novatel_gps_driver rather than a bare library name.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# Only what the installed headers need; the rest is linked PRIVATE above.
ament_export_dependencies(
  Boost
  diagnostic_msgs
  diagnostic_updater
  gps_msgs
  nmea_msgs
  novatel_gps_msgs
  rclcpp
  rtcm_msgs
  sensor_msgs
  swri_serial_util
)

ament_package()
