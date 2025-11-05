cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Add executables
add_executable(demo_publisher_enhanced src/demo_publisher_enhanced.cpp)
add_executable(dummy_yaw_publisher src/dummy_yaw_publisher.cpp)

# Link dependencies
ament_target_dependencies(demo_publisher_enhanced
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(dummy_yaw_publisher
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  demo_publisher_enhanced
  dummy_yaw_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/px4_converter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()