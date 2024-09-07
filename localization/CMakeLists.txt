cmake_minimum_required(VERSION 3.5)
project(localization)

# Dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executable
add_executable(scan_matcher src/scan_matcher.cpp)

# Link libraries
target_link_libraries(scan_matcher
  ${OpenCV_LIBRARIES}
  yaml-cpp  # Link yaml-cpp explicitly
)

# Specify package dependencies
ament_target_dependencies(scan_matcher
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
)

install(TARGETS
  scan_matcher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
