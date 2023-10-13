cmake_minimum_required(VERSION 3.5)
project(urdf-models)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf textures
  DESTINATION share/${PROJECT_NAME}
)

ament_package()