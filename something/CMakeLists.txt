cmake_minimum_required(VERSION 3.8)
project(bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(ma_utils REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY 
  config 
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  scripts/static
  scripts/templates
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
