cmake_minimum_required(VERSION 3.8)
project(ros_deep_grasp)

# Default to C++17
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies 

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


## Include directories
include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
)

install(DIRECTORY ros_deep_grasp/ros_deep_grasp_module/
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY ros_deep_grasp/ros_deep_grasp_module/lib/utils
  DESTINATION share/${PROJECT_NAME}/utils
)
install(DIRECTORY ros_deep_grasp/ros_deep_grasp_module/lib/model
  DESTINATION share/${PROJECT_NAME}/model
)


install(PROGRAMS
  ros_deep_grasp/service_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()





  
