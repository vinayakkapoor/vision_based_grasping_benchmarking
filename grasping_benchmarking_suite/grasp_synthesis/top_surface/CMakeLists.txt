cmake_minimum_required(VERSION 3.8)
project(top_surface)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  segmentation
  surface
  io
  search
  sample_consensus
  visualization
)
find_package(grasp_interfaces REQUIRED)

# Create executable
add_executable(grasp_synthesis_node
  top_surface/grasp_synthesis.cpp
)

# Include directories
target_include_directories(grasp_synthesis_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(grasp_synthesis_node
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  grasp_interfaces
)

install(PROGRAMS
  top_surface/pyefd.py
  top_surface/graspSynthesis.py
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(grasp_synthesis_node
  ${PCL_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
)

# Install and package
install(TARGETS grasp_synthesis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()