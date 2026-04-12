/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi
#include "seed_r7_ros_controller/seed_r7_mover_controller.h"
#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

robot_hardware::MoverController::MoverController(
  rclcpp::Node::SharedPtr node, robot_hardware::RobotHW* _in_hw)
  : node_(node), hw_(_in_hw),
    vx_(0), vy_(0), vth_(0), x_(0), y_(0), th_(0),
    en_x_(0), en_y_(0), en_th_(0)
{
  // Read parameters
  auto declare_if_missing = [&](const std::string& name, double def) {
    if (!node_->has_parameter(name)) node_->declare_parameter(name, def);
  };
  auto declare_bool_if_missing = [&](const std::string& name, bool def) {
    if (!node_->has_parameter(name)) node_->declare_parameter(name, def);
  };

  declare_bool_if_missing("seed_r7_mover_controller.encoder_odom", false);
  declare_if_missing("seed_r7_mover_controller.wheel_radius", 0.075);
  declare_if_missing("seed_r7_mover_controller.tread", 0.39);
  declare_if_missing("seed_r7_mover_controller.wheelbase", 0.453);
  declare_if_missing("seed_r7_mover_controller.ros_rate", 50.0);
  declare_if_missing("seed_r7_mover_controller.odom_rate", 0.02);
  declare_if_missing("seed_r7_mover_controller.safety_rate", 0.05);
  declare_if_missing("seed_r7_mover_controller.safety_duration", 0.5);

  encoder_odom_ = node_->get_parameter("seed_r7_mover_controller.encoder_odom").as_bool();

  float wheel_radius = static_cast<float>(
    node_->get_parameter("seed_r7_mover_controller.wheel_radius").as_double());
  float tread = static_cast<float>(
    node_->get_parameter("seed_r7_mover_controller.tread").as_double());
  float wheelbase = static_cast<float>(
    node_->get_parameter("seed_r7_mover_controller.wheelbase").as_double());

  // Compute kinematic coefficients
  k1_ = -sqrt(2.0f) * (sqrt(pow(tread, 2) + pow(wheelbase, 2)) / 2.0f) *
        sin(M_PI / 4.0f + atan2(tread / 2.0f, wheelbase / 2.0f)) / wheel_radius;
  k2_ = 1.0f / wheel_radius;
  k3_ = wheel_radius / 4.0f;
  k4_ = (tread / 2.0f) * sqrt(1.0f + pow(wheelbase / 2.0f, 2) / pow(tread / 2.0f, 2)) *
        wheel_radius /
        (4.0f * ((tread / 2.0f) + (wheelbase / 2.0f)) *
         (sqrt(pow(tread, 2) + pow(wheelbase, 2)) / 2.0f));

  ros_rate_       = node_->get_parameter("seed_r7_mover_controller.ros_rate").as_double();
  odom_rate_      = node_->get_parameter("seed_r7_mover_controller.odom_rate").as_double();
  safety_rate_    = node_->get_parameter("seed_r7_mover_controller.safety_rate").as_double();
  safety_duration_ = node_->get_parameter("seed_r7_mover_controller.safety_duration").as_double();

  // Wheel aero_index
  if (!node_->has_parameter("joint_settings.wheel.aero_index")) {
    node_->declare_parameter("joint_settings.wheel.aero_index", std::vector<int64_t>{});
  }
  auto aero_index_64 = node_->get_parameter("joint_settings.wheel.aero_index").as_integer_array();
  aero_index_.assign(aero_index_64.begin(), aero_index_64.end());
  num_of_wheels_ = static_cast<int>(aero_index_.size());

  servo_on_ = false;
  current_time_ = node_->now();
  last_time_ = current_time_;
  time_stamp_ = current_time_;

  // TF broadcaster
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // Publishers
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  initialpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1);

  // Subscribers
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    std::bind(&MoverController::cmdVelCallback, this, std::placeholders::_1));

  // Timers
  auto safety_ms = std::chrono::milliseconds(
    static_cast<int>(safety_rate_ * 1000));
  safe_timer_ = node_->create_wall_timer(
    safety_ms, std::bind(&MoverController::safetyCheckCallback, this));

  auto odom_ms = std::chrono::milliseconds(
    static_cast<int>(odom_rate_ * 1000));
  odom_timer_ = node_->create_wall_timer(
    odom_ms, std::bind(&MoverController::calculateOdometry, this));

  // Services
  led_control_server_ = node_->create_service<seed_r7_ros_controller::srv::LedControl>(
    "led_control",
    std::bind(&MoverController::ledControlCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  set_initialpose_server_ = node_->create_service<seed_r7_ros_controller::srv::SetInitialPose>(
    "set_initialpose",
    std::bind(&MoverController::setInitialPoseCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  reset_odom_server_ = node_->create_service<seed_r7_ros_controller::srv::ResetOdom>(
    "reset_odom",
    std::bind(&MoverController::resetOdomCallback, this,
              std::placeholders::_1, std::placeholders::_2));
}

robot_hardware::MoverController::~MoverController()
{
}

void robot_hardware::MoverController::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr _cmd_vel)
{
  rclcpp::Time now = node_->now();
  RCLCPP_DEBUG(node_->get_logger(), "cmd_vel: %f %f %f",
               _cmd_vel->linear.x, _cmd_vel->linear.y, _cmd_vel->angular.z);

  if (base_mtx_.try_lock()) {
    if (hw_->robot_status_.p_stopped_err_ ||
        (hw_->robot_status_.connection_err_ && hw_->robot_status_.calib_err_)) {
      vx_ = vy_ = vth_ = 0.0;
    } else if (_cmd_vel->linear.x == 0.0 && _cmd_vel->linear.y == 0.0 &&
               _cmd_vel->angular.z == 0.0) {
      vx_ = vy_ = vth_ = 0.0;
    } else {
      vx_  = _cmd_vel->linear.x;
      vy_  = _cmd_vel->linear.y;
      vth_ = _cmd_vel->angular.z;
    }

    RCLCPP_DEBUG(node_->get_logger(), "act_vel: %f %f %f", vx_, vy_, vth_);

    if (!servo_on_) {
      servo_on_ = true;
      hw_->onWheelServo(servo_on_);
    }

    std::vector<int16_t> wheel_vel(num_of_wheels_);
    velocityToWheel(vx_, vy_, vth_, wheel_vel);
    hw_->turnWheel(wheel_vel);

    time_stamp_ = now;
    base_mtx_.unlock();
  } else {
    RCLCPP_WARN(node_->get_logger(), "cmd_vel comes before sending previous message");
  }
}

void robot_hardware::MoverController::safetyCheckCallback()
{
  rclcpp::Time now = node_->now();
  if ((now - time_stamp_).seconds() >= safety_duration_ && servo_on_) {
    std::vector<int16_t> wheel_velocity(num_of_wheels_, 0);
    vx_ = vy_ = vth_ = 0.0;
    RCLCPP_WARN(node_->get_logger(), "Base: safety stop");
    hw_->turnWheel(wheel_velocity);
    servo_on_ = false;
    hw_->onWheelServo(servo_on_);
  }
}

void robot_hardware::MoverController::calculateEncoderOdometry()
{
  double dt = (current_time_ - last_time_).seconds();
  double en_vx, en_vy, en_vth;
  double theta = 0.0;
  double cos_th = cos(theta);
  double sin_th = sin(theta);

  double v1, v2, v3, v4;
  if (std::isfinite(hw_->wheel_velocities_.at(0)) && std::isfinite(hw_->wheel_velocities_.at(1)) &&
      std::isfinite(hw_->wheel_velocities_.at(2)) && std::isfinite(hw_->wheel_velocities_.at(3))) {
    v1 = hw_->wheel_velocities_.at(0);  // front right
    v2 = hw_->wheel_velocities_.at(2);  // front left
    v3 = hw_->wheel_velocities_.at(3);  // rear left
    v4 = hw_->wheel_velocities_.at(1);  // rear right
  } else {
    v1 = v2 = v3 = v4 = 0.0;
  }

  en_vx  = k3_ * ((-cos_th - sin_th) * v1 + ( cos_th - sin_th) * v2 +
                   ( cos_th + sin_th) * v3 + (-cos_th + sin_th) * v4);
  en_vy  = k3_ * ((-cos_th + sin_th) * v1 + (-cos_th - sin_th) * v2 +
                   ( cos_th - sin_th) * v3 + ( cos_th + sin_th) * v4);
  en_vth = -k4_ * (v1 + v2 + v3 + v4);

  double delta_x  = (en_vx * cos(en_th_) - en_vy * sin(en_th_)) * dt;
  double delta_y  = (en_vx * sin(en_th_) + en_vy * cos(en_th_)) * dt;
  double delta_th = en_vth * dt;

  en_x_  += delta_x;
  en_y_  += delta_y;
  en_th_ += delta_th;

  // Build quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0, 0, en_th_);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  // TF: odom → base_link
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp    = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "base_link";
  odom_trans.transform.translation.x = en_x_;
  odom_trans.transform.translation.y = en_y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation      = odom_quat;
  odom_broadcaster_->sendTransform(odom_trans);

  // Odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = current_time_;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";
  odom.pose.pose.position.x    = en_x_;
  odom.pose.pose.position.y    = en_y_;
  odom.pose.pose.position.z    = 0.0;
  odom.pose.pose.orientation   = odom_quat;
  odom.twist.twist.linear.x    = en_vx;
  odom.twist.twist.linear.y    = en_vy;
  odom.twist.twist.angular.z   = en_vth;
  odom_pub_->publish(odom);
}

void robot_hardware::MoverController::calculateCmdVelOdometry()
{
  double dt = (current_time_ - last_time_).seconds();

  double delta_x  = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
  double delta_y  = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
  double delta_th = vth_ * dt;

  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;

  tf2::Quaternion q;
  q.setRPY(0, 0, th_);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp    = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "base_link";
  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation      = odom_quat;
  odom_broadcaster_->sendTransform(odom_trans);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = current_time_;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";
  odom.pose.pose.position.x  = x_;
  odom.pose.pose.position.y  = y_;
  odom.pose.pose.position.z  = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x  = vx_;
  odom.twist.twist.linear.y  = vy_;
  odom.twist.twist.angular.z = vth_;
  odom_pub_->publish(odom);
}

void robot_hardware::MoverController::calculateOdometry()
{
  current_time_ = node_->now();

  if (hw_->lower_connected_) {
    if (encoder_odom_) calculateEncoderOdometry();
    else calculateCmdVelOdometry();
  } else {
    calculateCmdVelOdometry();
  }

  last_time_ = current_time_;
}

void robot_hardware::MoverController::velocityToWheel(
  double _linear_x, double _linear_y, double _angular_z,
  std::vector<int16_t>& _wheel_vel)
{
  float theta = 0.0f;
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  float dy     = static_cast<float>(_linear_x * cos_theta - _linear_y * sin_theta);
  float dx     = static_cast<float>(_linear_x * sin_theta + _linear_y * cos_theta);
  float dtheta = static_cast<float>(_angular_z);

  float v1 = k1_ * dtheta + k2_ * ((-cos_theta + sin_theta) * dx + (-cos_theta - sin_theta) * dy);
  float v2 = k1_ * dtheta + k2_ * ((-cos_theta - sin_theta) * dx + ( cos_theta - sin_theta) * dy);
  float v3 = k1_ * dtheta + k2_ * (( cos_theta - sin_theta) * dx + ( cos_theta + sin_theta) * dy);
  float v4 = k1_ * dtheta + k2_ * (( cos_theta + sin_theta) * dx + (-cos_theta + sin_theta) * dy);

  _wheel_vel[0] = static_cast<int16_t>(v2 * (180.0f / M_PI));  // front left
  _wheel_vel[1] = static_cast<int16_t>(v1 * (180.0f / M_PI));  // front right
  _wheel_vel[2] = static_cast<int16_t>(v3 * (180.0f / M_PI));  // rear left
  _wheel_vel[3] = static_cast<int16_t>(v4 * (180.0f / M_PI));  // rear right
}

void robot_hardware::MoverController::setInitialPoseCallback(
  const seed_r7_ros_controller::srv::SetInitialPose::Request::SharedPtr _req,
  seed_r7_ros_controller::srv::SetInitialPose::Response::SharedPtr _res)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _req->theta);
  geometry_msgs::msg::Quaternion pose_quat = tf2::toMsg(q);

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp    = node_->now();
  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose.position.x  = _req->x;
  initial_pose.pose.pose.position.y  = _req->y;
  initial_pose.pose.pose.position.z  = 0.0;
  initial_pose.pose.pose.orientation = pose_quat;
  initial_pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  initial_pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  initial_pose.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;

  initialpose_pub_->publish(initial_pose);
  _res->result = "SetInitialPose succeeded";
}

void robot_hardware::MoverController::ledControlCallback(
  const seed_r7_ros_controller::srv::LedControl::Request::SharedPtr _req,
  seed_r7_ros_controller::srv::LedControl::Response::SharedPtr _res)
{
  hw_->runLedScript(_req->send_number, _req->script_number);
  _res->result = "LED succeeded";
}

void robot_hardware::MoverController::resetOdomCallback(
  const seed_r7_ros_controller::srv::ResetOdom::Request::SharedPtr /*_req*/,
  seed_r7_ros_controller::srv::ResetOdom::Response::SharedPtr _res)
{
  x_ = y_ = th_ = 0.0;
  en_x_ = en_y_ = en_th_ = 0.0;
  _res->result = "ResetOdom succeeded";
}
