cmake_minimum_required(VERSION 3.8)
project(seed_r7_robot_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(angles REQUIRED)
find_package(seed_r7_ros_controller REQUIRED)

# ---- typef ----
add_library(${PROJECT_NAME}_typef SHARED typef/src/stroke_converter.cpp)
target_include_directories(${PROJECT_NAME}_typef PRIVATE typef/src)
ament_target_dependencies(${PROJECT_NAME}_typef rclcpp pluginlib seed_r7_ros_controller)

# ---- typeg ----
add_library(${PROJECT_NAME}_typeg SHARED typeg/src/stroke_converter.cpp)
target_include_directories(${PROJECT_NAME}_typeg PRIVATE typeg/src)
ament_target_dependencies(${PROJECT_NAME}_typeg rclcpp pluginlib seed_r7_ros_controller)

# ---- typeg_arm ----
add_library(${PROJECT_NAME}_typeg_arm SHARED typeg_arm/src/stroke_converter.cpp)
target_include_directories(${PROJECT_NAME}_typeg_arm PRIVATE typeg_arm/src)
ament_target_dependencies(${PROJECT_NAME}_typeg_arm rclcpp pluginlib seed_r7_ros_controller)

# ---- typeg2_arm ----
add_library(${PROJECT_NAME}_typeg2_arm SHARED typeg2_arm/src/stroke_converter.cpp)
target_include_directories(${PROJECT_NAME}_typeg2_arm PRIVATE typeg2_arm/src)
ament_target_dependencies(${PROJECT_NAME}_typeg2_arm rclcpp pluginlib seed_r7_ros_controller)

# Register stroke converter plugins against seed_r7_ros_controller
pluginlib_export_plugin_description_file(seed_r7_ros_controller typef/plugin_description.xml)
pluginlib_export_plugin_description_file(seed_r7_ros_controller typeg/plugin_description.xml)
pluginlib_export_plugin_description_file(seed_r7_ros_controller typeg_arm/plugin_description.xml)
pluginlib_export_plugin_description_file(seed_r7_ros_controller typeg2_arm/plugin_description.xml)

install(
  TARGETS
    ${PROJECT_NAME}_typef
    ${PROJECT_NAME}_typeg
    ${PROJECT_NAME}_typeg_arm
    ${PROJECT_NAME}_typeg2_arm
  DESTINATION lib
)

# Install config directories (exclude source files)
install(
  DIRECTORY typef typeg typeg_arm typeg2_arm
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*/src" EXCLUDE
)

ament_package()
