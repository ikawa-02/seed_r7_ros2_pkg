cmake_minimum_required(VERSION 3.8)
project(seed_r7_gazebo)

# Use C++17 for gz-sim8
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ----- Gazebo Sim 8 (Harmonic) via vendor packages -----
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)

# ----- MimicPlugin: pure gz-sim, no ROS deps -----
add_library(mimic_plugin SHARED
  src/MimicPlugin.cpp
)
target_compile_features(mimic_plugin PUBLIC cxx_std_17)
target_link_libraries(mimic_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2-register
  gz-math7::gz-math7
)

# ----- GazeboRosPlanarForceMove: gz-sim8 + ROS2 -----
add_library(gazebo_ros_planar_force_move SHARED
  src/gazebo_ros_planar_move.cpp
)
target_compile_features(gazebo_ros_planar_force_move PUBLIC cxx_std_17)
target_link_libraries(gazebo_ros_planar_force_move
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2-register
  gz-math7::gz-math7
)
ament_target_dependencies(gazebo_ros_planar_force_move
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS mimic_plugin gazebo_ros_planar_force_move
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
