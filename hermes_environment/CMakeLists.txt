cmake_minimum_required(VERSION 3.5)
project(hermes_environment)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(irobot_create_control REQUIRED)
find_package(hermes_create_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Create a shared library for the component
add_library(rf_beacon_publisher_lib SHARED)
target_sources(
  rf_beacon_publisher_lib
  PRIVATE
    src/rf_beacon_publisher.cpp
)
target_include_directories(rf_beacon_publisher_lib PUBLIC include)
ament_target_dependencies(rf_beacon_publisher_lib
  rclcpp
  gazebo_msgs
  std_msgs
  rclcpp_components
)

rclcpp_components_register_node(rf_beacon_publisher_lib
  PLUGIN "RFBeaconPublisher"
  EXECUTABLE rf_beacon_publisher_node
)

# Install the library
install(TARGETS rf_beacon_publisher_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the other directories
install(
  DIRECTORY
    config
    launch
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()

ament_package()
