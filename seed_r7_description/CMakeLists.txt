cmake_minimum_required(VERSION 3.8)
project(seed_r7_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf typef csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
