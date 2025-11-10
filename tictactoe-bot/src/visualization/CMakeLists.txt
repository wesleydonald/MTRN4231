cmake_minimum_required(VERSION 3.8)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(visualization_node src/visualization.cpp)

ament_target_dependencies(visualization_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

