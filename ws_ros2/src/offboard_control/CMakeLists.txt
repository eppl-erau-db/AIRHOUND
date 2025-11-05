cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Minimal working set for E2E test
add_executable(px4_converter_node_simple src/px4_converter_node_simple.cpp)
add_executable(dummy_yaw_publisher src/dummy_yaw_publisher.cpp)

# Link dependencies
ament_target_dependencies(px4_converter_node_simple
  rclcpp
  px4_msgs
  std_msgs
  geometry_msgs
)

ament_target_dependencies(dummy_yaw_publisher
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  px4_converter_node_simple
  dummy_yaw_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
