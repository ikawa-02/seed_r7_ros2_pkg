#ifndef _ROBOT_HW_H_
#define _ROBOT_HW_H_

// ROS2 / ros2_control
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

// Messages
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

// pluginlib
#include <pluginlib/class_loader.hpp>

// URDF
#include <urdf/model.h>

// Diagnostics
#include <diagnostic_updater/diagnostic_updater.hpp>

// AERO controllers
#include "seed_r7_ros_controller/seed_r7_upper_controller.h"
#include "seed_r7_ros_controller/seed_r7_lower_controller.h"
#include "seed_r7_ros_controller/stroke_converter_base.h"

// Generated service types
#include "seed_r7_ros_controller/srv/reset_robot_status.hpp"
#include "seed_r7_ros_controller/msg/robot_info.hpp"

#include <mutex>
#include <thread>

namespace robot_hardware
{

// Forward declarations
class MoverController;
class HandController;

class RobotHW : public hardware_interface::SystemInterface
{
public:
  RobotHW();
  virtual ~RobotHW();

  // hardware_interface::SystemInterface overrides
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Public hardware access methods (used by MoverController/HandController)
  void readPos(const rclcpp::Time & time, const rclcpp::Duration & period, bool update);
  void runHandScript(uint8_t _number, uint16_t _script, uint8_t _current);
  void turnWheel(std::vector<int16_t> & _vel);
  void onWheelServo(bool _value);
  void runLedScript(uint8_t _number, uint16_t _script);

  // Callbacks from cmd_vel / odom subscriptions
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr _cmd_vel);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr _odom);

  double getPeriod() { return ((double)CONTROL_PERIOD_US_) / (1000 * 1000); }

  // Public state
  bool comm_err_;
  struct RobotStatus {
    bool connection_err_;
    bool calib_err_;
    bool motor_err_;
    bool temp_err_;
    bool res_err_;
    bool step_out_err_;
    bool p_stopped_err_;
    bool power_err_;
  } robot_status_;

  std::vector<double> wheel_angles_;
  std::vector<double> wheel_velocities_;
  float wheel_vel_limit_;
  bool upper_connected_, lower_connected_;
  bool pub_robot_info_;

private:
  void getBatteryVoltage();
  void pubRobotInfo();
  void setRobotStatus();
  void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void resetRobotStatusCallback(
    const seed_r7_ros_controller::srv::ResetRobotStatus::Request::SharedPtr _req,
    seed_r7_ros_controller::srv::ResetRobotStatus::Response::SharedPtr _res);

protected:
  enum ControlMethod { EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID };
  enum JointType { NONE, PRISMATIC, ROTATIONAL, CONTINUOUS, FIXED };

  unsigned int number_of_angles_;
  std::vector<std::string> joint_list_;
  std::vector<JointType> joint_types_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;

  std::vector<double> prev_ref_strokes_;
  std::vector<int16_t> upper_act_strokes_;
  std::vector<int16_t> lower_act_strokes_;

  std::shared_ptr<robot_hardware::UpperController> controller_upper_;
  std::shared_ptr<robot_hardware::LowerController> controller_lower_;

  bool initialized_flag_;
  bool upper_send_enable_;

  int CONTROL_PERIOD_US_;
  float OVERLAP_SCALE_;

  std::mutex mutex_lower_;
  std::mutex mutex_upper_;

  std::vector<std::string> joint_names_upper_;
  std::vector<std::string> joint_names_lower_;
  std::string robot_model_plugin_;

  // pluginlib for stroke converter
  pluginlib::ClassLoader<seed_converter::StrokeConverter> converter_loader_;
  std::shared_ptr<seed_converter::StrokeConverter> stroke_converter_;

  // Internal rclcpp node for pub/sub/services within the hardware plugin
  rclcpp::Node::SharedPtr hw_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  bool executor_running_;

  // Battery voltage publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bat_vol_pub_;
  rclcpp::TimerBase::SharedPtr bat_vol_timer_;

  // Robot info publisher
  rclcpp::Publisher<seed_r7_ros_controller::msg::RobotInfo>::SharedPtr robot_info_pub_;
  rclcpp::TimerBase::SharedPtr robot_info_timer_;
  seed_r7_ros_controller::msg::RobotInfo robot_info_;

  // cmd_vel / odom subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // reset_robot_status service
  rclcpp::Service<seed_r7_ros_controller::srv::ResetRobotStatus>::SharedPtr reset_robot_status_server_;

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  int8_t pre_diag_level_;
  std::string pre_diag_msg_;

  // MoverController and HandController (owned by plugin, use hw_node_)
  std::shared_ptr<MoverController> mover_controller_;
  std::shared_ptr<HandController> hand_controller_;
};

}  // namespace robot_hardware

#endif  // _ROBOT_HW_H_
