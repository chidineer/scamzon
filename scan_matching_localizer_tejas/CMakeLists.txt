cmake_minimum_required(VERSION 3.8)
project(scan_matching_localizer_tejas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Ensure nav_msgs is found
find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)  # Ensure YAML-CPP is found
find_package(std_msgs REQUIRED)  # Add std_msgs to find Float64 message
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}  # Include YAML-CPP directories
)

# Add the executable
add_executable(scan_to_image_node src/scan_to_image_node.cpp)

# Link against the required libraries
ament_target_dependencies(
  scan_to_image_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  OpenCV
  YAML-CPP
  std_msgs  # Add std_msgs for publishing Float64 messages
  tf2  # Add tf2 for quaternion processing
  tf2_geometry_msgs  # For working with geometry messages and tf2
)

target_link_libraries(
  scan_to_image_node
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Install the executable
install(TARGETS
  scan_to_image_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
