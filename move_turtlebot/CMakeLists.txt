cmake_minimum_required(VERSION 3.5)
project(move_turtlebot)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create the executable for the node
add_executable(move_robot_node src/move_robot_node.cpp)

# Link libraries
ament_target_dependencies(move_robot_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

# Install targets
install(TARGETS
  move_robot_node
  DESTINATION lib/${PROJECT_NAME})

# # Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch)

# # Install RViz configuration files (if any)
# install(DIRECTORY rviz
#   DESTINATION share/${PROJECT_NAME}/rviz)

# # Install params files (if any)
# install(DIRECTORY params
#   DESTINATION share/${PROJECT_NAME}/params)

# Mark executable as a test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for installation
ament_package()
