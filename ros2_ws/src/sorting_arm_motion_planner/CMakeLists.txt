cmake_minimum_required(VERSION 3.8)
project(sorting_arm_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motion_planner src/motion_planner.cpp)

ament_target_dependencies(
  motion_planner
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
)

add_executable(planning_scene_setup src/planning_scene_setup.cpp)

ament_target_dependencies(
  planning_scene_setup
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
)

install(
  TARGETS
    motion_planner
    planning_scene_setup
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
