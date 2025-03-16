cmake_minimum_required(VERSION 3.8)
project(backend)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(backend_node src/backend_node.cpp)
ament_target_dependencies(backend_node
  rclcpp
  std_msgs
)

install(TARGETS
  backend_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
