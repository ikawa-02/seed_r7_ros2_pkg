cmake_minimum_required(VERSION 3.8)
project(seed_r7_navigation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/dummy_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
