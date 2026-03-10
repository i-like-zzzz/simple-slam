cmake_minimum_required(VERSION 3.8)
project(simple_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(simple_slam_core
  src/backend/pose_graph_backend.cpp
  src/simple_slam_node.cpp
  src/frontend/local_slam_frontend.cpp
  src/mapping/submap_2d.cpp
  src/optimization/pose_graph_2d.cpp
)

ament_target_dependencies(simple_slam_core
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(simple_slam_node src/simple_slam_node_main.cpp)
target_link_libraries(simple_slam_node simple_slam_core)
ament_target_dependencies(simple_slam_node rclcpp)
target_link_libraries(simple_slam_core ${PCL_LIBRARIES})

install(
  TARGETS
    simple_slam_core
    simple_slam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config docs rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
