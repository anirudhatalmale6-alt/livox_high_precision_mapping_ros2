cmake_minimum_required(VERSION 3.8)
project(livox_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

add_executable(livox_mapping_node src/livox_mapping_node.cpp)
ament_target_dependencies(livox_mapping_node
  rclcpp sensor_msgs std_msgs nav_msgs geometry_msgs tf2 tf2_ros pcl_conversions)
target_include_directories(livox_mapping_node PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(livox_mapping_node ${PCL_LIBRARIES})

install(TARGETS livox_mapping_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
