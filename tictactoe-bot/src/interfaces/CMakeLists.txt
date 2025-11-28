cmake_minimum_required(VERSION 3.8)
project(interfaces)

# --- find packages ---
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# --- generate service interfaces ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardPose.msg"
  "srv/MoveArm.srv"
  "srv/CloseGripper.srv"
  DEPENDENCIES geometry_msgs std_srvs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
