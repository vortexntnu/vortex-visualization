cmake_minimum_required(VERSION 3.8)
project(auv_gui)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/${PROJECT_NAME}_node.py
  ${PROJECT_NAME}/widgets.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
