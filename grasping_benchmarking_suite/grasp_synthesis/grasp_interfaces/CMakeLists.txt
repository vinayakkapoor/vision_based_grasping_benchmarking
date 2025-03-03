cmake_minimum_required(VERSION 3.8)
project(grasp_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Grasp.msg"
  "msg/Grasp2D.msg"
  "srv/ProcessAndExecute.srv"
  "srv/GraspPrediction.srv"
  "srv/Grasp2DPrediction.srv"
  "srv/EndEffectorWaypoint.srv"
  "srv/GripperCommand.srv"
  "srv/CurrentPose.srv"
  "srv/SetJointVelocity.srv"
  "srv/MoveJointRelative.srv"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
)
include_directories(
  include
  # ${PCL_INCLUDE_DIRS}
)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 

