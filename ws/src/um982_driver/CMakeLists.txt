cmake_minimum_required(VERSION 3.8)
project(um982_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(nmea_parser src/nmea_parser.cpp)
target_include_directories(nmea_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(um982_driver_node src/um982_driver_node.cpp)
target_include_directories(um982_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(um982_driver_node nmea_parser)
ament_target_dependencies(um982_driver_node
  rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS um982_driver_node nmea_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nmea_parser test/test_nmea_parser.cpp)
  target_include_directories(test_nmea_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_nmea_parser nmea_parser)

  # Header-only MAVLink RTCM extractor — no library to link.
  ament_add_gtest(test_mavlink_rtcm test/test_mavlink_rtcm.cpp)
  target_include_directories(test_mavlink_rtcm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
