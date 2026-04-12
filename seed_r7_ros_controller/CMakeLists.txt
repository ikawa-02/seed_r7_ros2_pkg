cmake_minimum_required(VERSION 3.8)
project(seed_r7_ros_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(angles REQUIRED)
find_package(seed_smartactuator_sdk REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Messages and Services ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriverStatus.msg"
  "msg/Pose.msg"
  "msg/RobotInfo.msg"
  "msg/RobotStatus.msg"
  "srv/HandControl.srv"
  "srv/LedControl.srv"
  "srv/SetInitialPose.srv"
  "srv/ResetRobotStatus.srv"
  "srv/ResetOdom.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# ---- Hardware Interface Plugin (SystemInterface) ----
add_library(seed_r7_hardware_interface SHARED
  src/seed_r7_robot_hardware.cpp
  src/seed_r7_upper_controller.cpp
  src/seed_r7_lower_controller.cpp
  src/stroke_converter_base.cpp
  src/seed_r7_mover_controller.cpp
  src/seed_r7_hand_controller.cpp
)
target_include_directories(seed_r7_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(seed_r7_hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  hardware_interface
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  diagnostic_updater
  angles
  seed_smartactuator_sdk
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(seed_r7_hardware_interface "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(hardware_interface seed_r7_hardware.xml)

# ---- Install ----
install(
  TARGETS seed_r7_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES seed_r7_hardware.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
