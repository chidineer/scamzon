cmake_minimum_required(VERSION 3.5)
project(cylinder_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# Add your node executable (ensure this is only defined once)
add_executable(cylinder_navigation_node src/cylinder_navigation_node.cpp)
add_executable(cylinder_detector src/cylinder_detector.cpp)
add_executable(place_cylinder src/place_cylinder.cpp)

# Link against required libraries
ament_target_dependencies(cylinder_navigation_node
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  std_msgs
  nav2_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
)
ament_target_dependencies(cylinder_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
)
ament_target_dependencies(place_cylinder
  rclcpp
  gazebo_msgs
  geometry_msgs
  std_msgs
)


install(TARGETS
  cylinder_navigation_node
  cylinder_detector
  place_cylinder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
