cmake_minimum_required(VERSION 3.5)
project(scamazon_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add the reach_goals executable
add_executable(reach_goals src/reach_goals.cpp src/main.cpp)

# Link dependencies for reach_goals
ament_target_dependencies(reach_goals
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Ensure that the include directory is added to the project
include_directories(
  include
)

# Add the path_optimizer library
add_library(path_optimizer src/path_optimizer.cpp)

# Link dependencies for path_optimizer
ament_target_dependencies(path_optimizer
  rclcpp
  geometry_msgs
)

# Make sure reach_goals is linked with path_optimizer
target_link_libraries(reach_goals path_optimizer)

# Install the reach_goals executable
install(TARGETS
  reach_goals
  DESTINATION lib/${PROJECT_NAME})

# Add the initial_pose_publisher executable
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)

# Link dependencies for initial_pose_publisher
ament_target_dependencies(initial_pose_publisher
  rclcpp
  geometry_msgs
)

# Install the initial_pose_publisher executable
install(TARGETS
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install the path_optimizer library
install(TARGETS
  path_optimizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export package
ament_package()
