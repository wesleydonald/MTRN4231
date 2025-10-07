cmake_minimum_required(VERSION 3.8)
project(lab3_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(move_to_marker src/move_to_marker.cpp)
target_include_directories(move_to_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_marker
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_ros"
)


install(TARGETS 
  move_to_marker
DESTINATION lib/${PROJECT_NAME})

# Install launch files.

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
