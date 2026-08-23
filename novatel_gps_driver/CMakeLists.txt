cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)

project(novatel_gps_driver)

# System dependencies
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(novatel_gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swri_math_util  REQUIRED)
find_package(swri_serial_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED CONFIG)

if(TARGET Boost::headers)
  set(BOOST_HEADERS_TARGET Boost::headers)
elseif(TARGET Boost::boost)
  set(BOOST_HEADERS_TARGET Boost::boost)
else()
  message(FATAL_ERROR "Boost headers target not found. Expected Boost::headers or Boost::boost.")
endif()

# libpcap; it doesn't provide PkgConfig or CMake modules
find_library(libpcap_LIBRARIES pcap)
if ("${libpcap_LIBRARIES}" STREQUAL "pcap-NOTFOUND")
  set(libpcap_FOUND FALSE)
else()
  set(libpcap_FOUND TRUE)
endif()

include_directories(include)

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
  src/parsers/parsing_utils.cpp
  src/parsers/psrdop2.cpp
  src/parsers/range.cpp
  src/parsers/time.cpp
  src/parsers/trackstat.cpp
  src/parsers/rxstatus.cpp
)
set_property(TARGET ${PROJECT_NAME}
  PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}
  ${BOOST_HEADERS_TARGET}
  ${libpcap_LIBRARIES}
  ${diagnostic_msgs_TARGETS}
  ${diagnostic_updater_TARGETS}
  ${gps_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${novatel_gps_msgs_TARGETS}
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  swri_math_util::swri_math_util
  swri_serial_util::swri_serial_util
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
target_link_libraries(${PROJECT_NAME}_nodes
  ${PROJECT_NAME}
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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

### Install Launch Files ###
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_nodes
  libpcap
)

ament_package()
