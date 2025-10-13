cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (minimal set for testing)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add simple test executables
add_executable(dummy_yaw_publisher src/dummy_yaw_publisher.cpp)
add_executable(simple_test_node src/simple_test_node.cpp)

# Link dependencies
ament_target_dependencies(dummy_yaw_publisher
  rclcpp
  std_msgs
)

ament_target_dependencies(simple_test_node
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  dummy_yaw_publisher
  simple_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()