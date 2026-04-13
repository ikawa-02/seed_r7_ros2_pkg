cmake_minimum_required(VERSION 3.8)
project(seed_r7_typeg_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
