cmake_minimum_required(VERSION 3.8)
project(my_gripper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
# find_package(joint_state_publisher_gui REQUIRED)  # Comment out for now
find_package(rviz2 REQUIRED)
find_package(xacro REQUIRED)

# Export dependencies
ament_export_dependencies(robot_state_publisher rviz2 xacro)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  "package.xml"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()