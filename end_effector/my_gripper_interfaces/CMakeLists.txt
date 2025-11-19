cmake_minimum_required(VERSION 3.8)
project(my_gripper_interfaces)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This is the line that finds your .srv file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EndEffectorCmd.srv"
  # Add more .msg or .srv files here if you create them
  DEPENDENCIES std_msgs
)

ament_package()