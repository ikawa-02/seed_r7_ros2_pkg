#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

geometry_msgs::msg::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quat);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_sample_node");

  // spin in background so MoveIt callbacks are processed
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::msg::Pose target_pose1, target_pose2;

  // --- set registered position ---
  moveit::planning_interface::MoveGroupInterface move_group_arms(node, "upper_body");
  move_group_arms.setPoseReferenceFrame("base_link");
  move_group_arms.setNamedTarget("reset-pose");
  move_group_arms.move();

  // --- right arm without waist ---
  moveit::planning_interface::MoveGroupInterface move_group_right(node, "rarm");
  move_group_right.setEndEffectorLink("r_eef_grasp_link");
  move_group_right.setPoseReferenceFrame("base_link");

  target_pose1.orientation = rpy_to_geometry_quat(-1.57, 0.79, 0);
  target_pose1.position.x = 0.3;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 1.0;
  move_group_right.setPoseTarget(target_pose1);

  bool success = (move_group_right.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) { move_group_right.execute(my_plan); }

  // --- left arm without waist ---
  moveit::planning_interface::MoveGroupInterface move_group_left(node, "larm");
  move_group_left.setEndEffectorLink("l_eef_grasp_link");
  move_group_left.setPoseReferenceFrame("base_link");

  target_pose2.orientation = rpy_to_geometry_quat(1.57, 0.79, 0);
  target_pose2.position.x = 0.3;
  target_pose2.position.y = 0.2;
  target_pose2.position.z = 1.0;
  move_group_left.setPoseTarget(target_pose2);

  success = (move_group_left.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) { move_group_left.execute(my_plan); }

  // --- with both arms ---
  target_pose1.position.x += 0.2;
  target_pose2.position.x += 0.2;

  move_group_arms.setPoseTarget(target_pose1, "r_eef_grasp_link");
  move_group_arms.setPoseTarget(target_pose2, "l_eef_grasp_link");

  success = (move_group_arms.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) { move_group_arms.execute(my_plan); }

  // --- set registered position ---
  move_group_arms.setNamedTarget("reset-pose");
  move_group_arms.move();

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
