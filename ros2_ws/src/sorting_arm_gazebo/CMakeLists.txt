cmake_minimum_required(VERSION 3.8)
project(sorting_arm_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    models
    scripts
    worlds
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/randomize_objects.py
    scripts/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
