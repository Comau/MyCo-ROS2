cmake_minimum_required(VERSION 3.8)
project(myco_10_1000mm_cartesian_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(cartesian_3_positions_node
  src/cartesian_3_positions_node.cpp
)

ament_target_dependencies(cartesian_3_positions_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

install(TARGETS cartesian_3_positions_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
