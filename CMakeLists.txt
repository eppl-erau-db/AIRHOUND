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

# Add executables
add_executable(yaw_controller_node src/yaw_controller_node.cpp)
add_executable(dummy_yaw_publisher src/dummy_yaw_publisher.cpp)
add_executable(px4_simulator src/px4_simulator.cpp)
add_executable(simple_test_node src/simple_test_node.cpp)

# Link dependencies
ament_target_dependencies(yaw_controller_node
  rclcpp
  px4_msgs
  std_msgs
  geometry_msgs
)

ament_target_dependencies(dummy_yaw_publisher
  rclcpp
  std_msgs
)

ament_target_dependencies(px4_simulator
  rclcpp
  px4_msgs
)

ament_target_dependencies(simple_test_node
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  yaw_controller_node
  dummy_yaw_publisher
  px4_simulator
  simple_test_node
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