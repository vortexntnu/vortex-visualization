cmake_minimum_required(VERSION 3.8)
project(state_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(state_visualization_node
  src/state_visualization_node.cpp
  src/state_visualization_ros.cpp
)

ament_target_dependencies(state_visualization_node
  rclcpp
  geometry_msgs
  nav_msgs
  foxglove_msgs
  Eigen3
)

target_link_libraries(state_visualization_node
  Eigen3::Eigen
)

install(TARGETS
  state_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
