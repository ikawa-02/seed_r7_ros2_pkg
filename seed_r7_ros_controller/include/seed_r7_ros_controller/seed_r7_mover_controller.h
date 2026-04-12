/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#ifndef MOVER_CONTROLLER_H_
#define MOVER_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "seed_r7_ros_controller/srv/led_control.hpp"
#include "seed_r7_ros_controller/srv/set_initial_pose.hpp"
#include "seed_r7_ros_controller/srv/reset_odom.hpp"

// Forward declaration
namespace robot_hardware { class RobotHW; }

#define MAX_ACC_X 1.0
#define MAX_ACC_Y 1.0
#define MAX_ACC_Z 3.0


namespace robot_hardware
{

struct pose
{
  float x;
  float y;
  float theta;
};

class MoverController
{
public:
  explicit MoverController(rclcpp::Node::SharedPtr node, robot_hardware::RobotHW* _in_hw);
  ~MoverController();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr _cmd_vel);
  void safetyCheckCallback();
  void calculateOdometry();
  void velocityToWheel(double _linear_x, double _linear_y, double _angular_z,
                       std::vector<int16_t>& _wheel_vel);
  void setInitialPoseCallback(
    const seed_r7_ros_controller::srv::SetInitialPose::Request::SharedPtr _req,
    seed_r7_ros_controller::srv::SetInitialPose::Response::SharedPtr _res);
  void ledControlCallback(
    const seed_r7_ros_controller::srv::LedControl::Request::SharedPtr _req,
    seed_r7_ros_controller::srv::LedControl::Response::SharedPtr _res);
  void resetOdomCallback(
    const seed_r7_ros_controller::srv::ResetOdom::Request::SharedPtr _req,
    seed_r7_ros_controller::srv::ResetOdom::Response::SharedPtr _res);
  void calculateEncoderOdometry();
  void calculateCmdVelOdometry();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Time current_time_, last_time_, time_stamp_;
  rclcpp::TimerBase::SharedPtr odom_timer_, safe_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  rclcpp::Service<seed_r7_ros_controller::srv::LedControl>::SharedPtr led_control_server_;
  rclcpp::Service<seed_r7_ros_controller::srv::SetInitialPose>::SharedPtr set_initialpose_server_;
  rclcpp::Service<seed_r7_ros_controller::srv::ResetOdom>::SharedPtr reset_odom_server_;

  robot_hardware::RobotHW* hw_;

  double vx_, vy_, vth_, x_, y_, th_;
  double en_x_, en_y_, en_th_;
  double ros_rate_, odom_rate_, safety_rate_, safety_duration_;
  float k1_, k2_, k3_, k4_;
  int num_of_wheels_;
  bool servo_on_, encoder_odom_;
  std::vector<int> aero_index_;

  std::mutex base_mtx_;
};

}  // namespace robot_hardware

#endif
