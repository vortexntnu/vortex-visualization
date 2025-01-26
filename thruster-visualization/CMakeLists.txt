cmake_minimum_required(VERSION 3.8)
project(thruster_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

set(dependencies
tf2
tf2_ros
visualization_msgs
tf2_geometry_msgs
vortex_msgs
geometry_msgs
)

add_executable(${PROJECT_NAME}_node
  src/thruster_broadcaster.cpp
  src/thruster_visualization_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
