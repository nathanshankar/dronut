cmake_minimum_required(VERSION 3.5)
project(dronut_controller)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(drone_control_node
    src/main.cpp
    src/birotor_drone_controller.cpp
    src/joy_to_cmd_vel_node.cpp
)

# Link dependencies
ament_target_dependencies(drone_control_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)
target_include_directories(drone_control_node PRIVATE include)

# Install the executable
install(TARGETS drone_control_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
