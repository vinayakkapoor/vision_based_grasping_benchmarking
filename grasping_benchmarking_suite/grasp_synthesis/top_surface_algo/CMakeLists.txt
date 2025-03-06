cmake_minimum_required(VERSION 3.0.2)
project(top_surface_algo)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation  
)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Declare message and service files
add_message_files(
  FILES
  Grasp.msg
)

add_service_files(
  FILES
  GraspPrediction.srv
  EFDGrasp.srv
)

## Generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Build Executables
add_executable(grasp_synthesis scripts/grasp_synthesis.cpp)

## Fix add_dependencies
add_dependencies(grasp_synthesis ${catkin_EXPORTED_TARGETS})

## Link Libraries
target_link_libraries(grasp_synthesis ${catkin_LIBRARIES} ${PCL_LIBRARIES})

