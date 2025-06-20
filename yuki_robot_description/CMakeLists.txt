cmake_minimum_required(VERSION 3.8)
project(yuki_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Build the detected_dock_pose_publisher executable
add_executable(detected_dock_pose_publisher src/detected_dock_pose_publisher.cpp)
ament_target_dependencies(detected_dock_pose_publisher
  rclcpp
  geometry_msgs
  tf2_ros
)

# Install the executable
install(TARGETS
  detected_dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(
  DIRECTORY src launch rviz urdf config world meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
