cmake_minimum_required(VERSION 3.5)
project(autonomous_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(cylinder_detector src/cylinder_detector.cpp)
ament_target_dependencies(cylinder_detector rclcpp sensor_msgs gazebo_msgs nav_msgs tf2 tf2_geometry_msgs)

add_library(colour_detector src/colour_detector.cpp)
ament_target_dependencies(colour_detector rclcpp sensor_msgs cv_bridge OpenCV)

# Add executables and link necessary dependencies
add_executable(autonomous_mapping src/autonomous_mapping.cpp src/autonomous_mapping_main.cpp)
ament_target_dependencies(autonomous_mapping rclcpp sensor_msgs nav_msgs geometry_msgs nav2_msgs rtabmap_ros std_srvs OpenCV)

add_executable(localizer_and_navigation src/localizer_and_navigation.cpp src/localizer_and_navigation_main.cpp)
ament_target_dependencies(localizer_and_navigation rclcpp sensor_msgs nav_msgs geometry_msgs nav2_msgs rtabmap_ros std_srvs cv_bridge OpenCV)

target_link_libraries(localizer_and_navigation cylinder_detector colour_detector)

# Install the executables
install(TARGETS
  autonomous_mapping
  localizer_and_navigation
  cylinder_detector
  colour_detector
  DESTINATION lib/${PROJECT_NAME})

# ament_package() should come before the install commands for files
ament_package()

# Install the launch files, config files, and package.xml
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
