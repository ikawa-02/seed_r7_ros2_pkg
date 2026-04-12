cmake_minimum_required(VERSION 3.8)
project(seed_r7_samples)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Transitive deps of seed_r7_ros_controller exported targets
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(seed_r7_ros_controller REQUIRED)

# ---- moveit_sample_node ----
add_executable(moveit_sample_node src/moveit_sample.cpp)
ament_target_dependencies(moveit_sample_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)

# ---- record_robot_info_node ----
add_executable(record_robot_info_node src/record_robot_info.cpp)
ament_target_dependencies(record_robot_info_node
  rclcpp
  ament_index_cpp
  seed_r7_ros_controller
)

install(TARGETS
  moveit_sample_node
  record_robot_info_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Python scripts ----
install(PROGRAMS scripts/demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- config / launch ----
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
