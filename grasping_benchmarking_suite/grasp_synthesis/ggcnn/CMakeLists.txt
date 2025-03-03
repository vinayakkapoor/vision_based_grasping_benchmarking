cmake_minimum_required(VERSION 3.8)
project(ggcnn)

# Default to C++17
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies 

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# ## Generate messages and services
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Grasp2D.msg"
#   "srv/Grasp2DPrediction.srv"
#   DEPENDENCIES std_msgs geometry_msgs sensor_msgs
# )

## Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

install(DIRECTORY ggcnn/ggcnn_module/
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ggcnn/service_server.py
  DESTINATION lib/${PROJECT_NAME}
)




#ament_python_install_package(${PROJECT_NAME})
ament_package()





  
