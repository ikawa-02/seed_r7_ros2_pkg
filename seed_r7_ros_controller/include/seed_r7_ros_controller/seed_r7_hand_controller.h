#ifndef _HAND_CONTROLLER_H_
#define _HAND_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include "seed_r7_ros_controller/srv/hand_control.hpp"


// Forward declaration to avoid circular include
namespace robot_hardware { class RobotHW; }


namespace robot_hardware
{

class HandController
{
public:
  HandController(rclcpp::Node::SharedPtr node, robot_hardware::RobotHW* _in_hw);
  ~HandController();

  bool HandControlCallback(
    const seed_r7_ros_controller::srv::HandControl::Request::SharedPtr _req,
    seed_r7_ros_controller::srv::HandControl::Response::SharedPtr _res);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<seed_r7_ros_controller::srv::HandControl>::SharedPtr grasp_control_server_;
  robot_hardware::RobotHW* hw_;

  int right_number_;
  int left_number_;

  const uint16_t SCRIPT_GRASP   = 2;
  const uint16_t SCRIPT_UNGRASP = 3;
  const uint16_t SCRIPT_CANCEL  = 4;
};

}  // namespace robot_hardware

#endif
