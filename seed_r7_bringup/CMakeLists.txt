cmake_minimum_required(VERSION 3.8)
project(seed_r7_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES rviz.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
