cmake_minimum_required(VERSION 3.8)
project(groundgrid)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#####################################
## Find ament and ROS2 packages    ##
#####################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

################################
## Find external dependencies ##
################################
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

###################################
## ament specific configuration  ##
###################################

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  cv_bridge
  image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#####################################################
## Build library, node and component of this package ##
#####################################################

## Declare GroundGrid library
add_library(${PROJECT_NAME}_lib SHARED
  src/GroundGrid.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ${dependencies}
)

## Declare GroundSegmentation library
add_library(${PROJECT_NAME}_groundsegmentation_lib SHARED
  src/GroundSegmentation.cpp
)
target_include_directories(${PROJECT_NAME}_groundsegmentation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_groundsegmentation_lib
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_groundsegmentation_lib
  ${PCL_LIBRARIES}
)

## Declare standalone executable
add_executable(${PROJECT_NAME}_node
  src/GroundGridNode.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_groundsegmentation_lib
)
ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

## Occupancy Map Generator node
add_executable(occupancy_map_generator
  src/OccupancyMapGenerator.cpp
)
target_link_libraries(occupancy_map_generator
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(occupancy_map_generator
  ${dependencies}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_groundsegmentation_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ${PROJECT_NAME}_node
    occupancy_map_generator
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

## Mark launch files for installation
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_groundsegmentation_lib
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
