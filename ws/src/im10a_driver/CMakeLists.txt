cmake_minimum_required(VERSION 3.8)
project(im10a_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(witmotion_parser src/witmotion_parser.cpp)
target_include_directories(witmotion_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(im10a_driver_node src/im10a_driver_node.cpp)
target_include_directories(im10a_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(im10a_driver_node witmotion_parser)
ament_target_dependencies(im10a_driver_node rclcpp sensor_msgs)

install(TARGETS im10a_driver_node witmotion_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_witmotion_parser test/test_witmotion_parser.cpp)
  target_include_directories(test_witmotion_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_witmotion_parser witmotion_parser)
endif()

ament_package()
