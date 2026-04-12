#include "seed_r7_ros_controller/seed_r7_hand_controller.h"
#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"


robot_hardware::HandController::HandController(
  rclcpp::Node::SharedPtr node, robot_hardware::RobotHW* _in_hw)
  : node_(node), hw_(_in_hw), right_number_(0), left_number_(0)
{
  RCLCPP_INFO(node_->get_logger(), "hand_control_server start");

  grasp_control_server_ = node_->create_service<seed_r7_ros_controller::srv::HandControl>(
    "hand_control",
    std::bind(&HandController::HandControlCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Get hand numbers from parameters
  if (!node_->has_parameter("joint_settings.hand.right_number")) {
    node_->declare_parameter("joint_settings.hand.right_number", 0);
  }
  if (!node_->has_parameter("joint_settings.hand.left_number")) {
    node_->declare_parameter("joint_settings.hand.left_number", 0);
  }
  right_number_ = static_cast<int>(node_->get_parameter("joint_settings.hand.right_number").as_int());
  left_number_  = static_cast<int>(node_->get_parameter("joint_settings.hand.left_number").as_int());

  // initialize script cancel on hands
  if (right_number_ != 0) hw_->runHandScript(right_number_, SCRIPT_CANCEL, 0);
  if (left_number_  != 0) hw_->runHandScript(left_number_,  SCRIPT_CANCEL, 0);

  RCLCPP_INFO(node_->get_logger(), "Initialized HandController");
}

robot_hardware::HandController::~HandController()
{
}

bool robot_hardware::HandController::HandControlCallback(
  const seed_r7_ros_controller::srv::HandControl::Request::SharedPtr _req,
  seed_r7_ros_controller::srv::HandControl::Response::SharedPtr _res)
{
  uint8_t send_number = 0;
  uint16_t script_number = 0;

  RCLCPP_INFO(node_->get_logger(), "Grasp callback start");

  // position selection
  if (_req->position == seed_r7_ros_controller::srv::HandControl::Request::POSITION_RIGHT) {
    send_number = static_cast<uint8_t>(right_number_);
  } else if (_req->position == seed_r7_ros_controller::srv::HandControl::Request::POSITION_LEFT) {
    send_number = static_cast<uint8_t>(left_number_);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "please input POSITION_RIGHT(0) or POSITION_LEFT(1).");
    _res->result = "service call failed";
    return false;
  }

  // script selection
  if (_req->script == "grasp") {
    script_number = SCRIPT_GRASP;
  } else if (_req->script == "release") {
    script_number = SCRIPT_UNGRASP;
  } else if (_req->script == "cancel") {
    script_number = SCRIPT_CANCEL;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "please input \"grasp\", \"release\" or \"cancel\".");
    _res->result = "service call failed";
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "motion: %s", _req->script.c_str());
  if (send_number != 0) hw_->runHandScript(send_number, script_number, _req->current);

  RCLCPP_INFO(node_->get_logger(), "End Grasp");
  _res->result = "grasp succeeded";
  return true;
}
