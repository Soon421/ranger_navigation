cmake_minimum_required(VERSION 3.8)
project(nav2_navigation)
cmake_policy(SET CMP0071 NEW)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find Qt5 for GUI
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)  # Enable Qt MOC compiler

# Find yaml-cpp for waypoint save/load
find_package(yaml-cpp REQUIRED)

# Generate custom service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveWaypoints.srv"
  "srv/LoadWaypoints.srv"
)

# goal_publisher executable
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  rclcpp
  geometry_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(goal_publisher yaml-cpp)

# goal_publisher_gui executable
# Include header file for Qt MOC
add_executable(goal_publisher_gui
  src/goal_publisher_gui.cpp
  include/nav2_navigation/goal_publisher_gui.hpp
)
ament_target_dependencies(goal_publisher_gui
  rclcpp
  std_srvs
)
target_link_libraries(goal_publisher_gui Qt5::Widgets)

# Link generated interfaces (use interfaces within the same package)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(goal_publisher "${cpp_typesupport_target}")
target_link_libraries(goal_publisher_gui "${cpp_typesupport_target}")

install(TARGETS
  goal_publisher
  goal_publisher_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts (RIL scenario runner, etc.)
install(PROGRAMS
  scripts/random_goal_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY map_2D
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
