cmake_minimum_required(VERSION 3.5)
project(target_tracking_visualization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_filtering REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)


add_executable(${PROJECT_NAME}_node 
  src/target_tracking_visualization_ros_node.cpp 
  src/target_tracking_visualization_ros.cpp 
)

# target_link_libraries( ${PROJECT_NAME}_node
#   Eigen3::Eigen
# )

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp 
  std_msgs 
  vortex_msgs
  foxglove_msgs
  geometry_msgs
  vortex_filtering
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
