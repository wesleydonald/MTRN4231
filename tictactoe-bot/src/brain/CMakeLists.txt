cmake_minimum_required(VERSION 3.8)
project(brain) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# --- include directories ---
include_directories(
  include
)

# --- build executable ---
add_executable(brain src/brain.cpp src/tictactoe.cpp)
ament_target_dependencies(brain
  rclcpp
  geometry_msgs
  std_srvs
  interfaces
  std_msgs
)

# --- install targets ---
install(TARGETS brain
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# --- ament package ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
