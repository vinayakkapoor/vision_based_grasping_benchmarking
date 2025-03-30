cmake_minimum_required(VERSION 3.8)
project(benchmarking_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(isaac_ros2_messages)

include_directories(
  include
  # ${PCL_INCLUDE_DIRS}
)
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# add_executable(grasp_synthesis_node benchmarking_grasp/top_surface/grasp_synthesis_node.cpp)

# ament_target_dependencies(grasp_synthesis_node 
#   rclcpp 
#   sensor_msgs 
#   tf2 
#   tf2_geometry_msgs 
#   tf2_ros 
#   visualization_msgs 
#   geometry_msgs
#   pcl_conversions
# )
# target_link_libraries(grasp_synthesis_node ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the package
install(PROGRAMS
  benchmarking_grasp/benchmarking_pipeline.py
  # grasp_synthesis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(
  DIRECTORY include/meshes
  DESTINATION share/${PROJECT_NAME}/meshes
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 