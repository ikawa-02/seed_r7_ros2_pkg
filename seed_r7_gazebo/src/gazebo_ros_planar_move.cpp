/*
 * gz-sim8 + ROS2 port of GazeboRosPlanarForceMove
 * Original Copyright 2013 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */

#include "gazebo_ros_planar_move.h"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/Util.hh>

#include <cmath>

GZ_ADD_PLUGIN(
  gazebo::GazeboRosPlanarForceMove,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  gazebo::GazeboRosPlanarForceMove,
  "gazebo::GazeboRosPlanarForceMove"
)

namespace gazebo
{

GazeboRosPlanarForceMove::GazeboRosPlanarForceMove() {}

GazeboRosPlanarForceMove::~GazeboRosPlanarForceMove()
{
  if (this->executor_) {
    this->executor_->cancel();
  }
  if (this->executor_thread_.joinable()) {
    this->executor_thread_.join();
  }
}

void GazeboRosPlanarForceMove::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  this->model_ = gz::sim::Model(_entity);

  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);

  auto get_str = [&](const std::string &tag, const std::string &def) -> std::string {
    if (sdf->HasElement(tag)) return sdf->GetElement(tag)->Get<std::string>();
    return def;
  };
  auto get_double = [&](const std::string &tag, double def) -> double {
    if (sdf->HasElement(tag)) return sdf->GetElement(tag)->Get<double>();
    return def;
  };
  auto get_bool = [&](const std::string &tag, bool def) -> bool {
    if (sdf->HasElement(tag)) return sdf->GetElement(tag)->Get<bool>();
    return def;
  };

  this->command_topic_    = get_str("commandTopic",    "cmd_vel");
  this->odometry_topic_   = get_str("odometryTopic",   "odom");
  this->odometry_frame_   = get_str("odometryFrame",   "odom");
  this->robot_base_frame_ = get_str("robotBaseFrame",  "base_link");
  this->odometry_rate_    = get_double("odometryRate", 20.0);
  this->enable_y_axis_    = get_bool("enableYAxis",    true);
  this->use_force_feedback_ = get_bool("useForceFeedback", false);

  // Get canonical link for velocity / pose queries
  this->canonical_link_entity_ = this->model_.CanonicalLink(_ecm);

  if (this->use_force_feedback_) {
    std::string link_name = get_str("appliedForceLink", "base_link");
    auto lnk = this->model_.LinkByName(_ecm, link_name);
    if (lnk == gz::sim::kNullEntity) {
      // fall back to canonical link
      lnk = this->canonical_link_entity_;
    }
    this->force_link_entity_ = lnk;
    this->gain_x_   = get_double("forceGainX",   20000);
    this->gain_y_   = get_double("forceGainY",   20000);
    this->gain_rot_ = get_double("forceGainRot", 5000);
    this->v_dead_zone_ = get_double("velocityDeadZone", 0.0001);
  }

  // Enable WorldPose + velocity components on canonical link so we can read them
  gz::sim::Link canonical(this->canonical_link_entity_);
  canonical.EnableVelocityChecks(_ecm, true);

  if (this->use_force_feedback_ &&
      this->force_link_entity_ != this->canonical_link_entity_)
  {
    gz::sim::Link force_lnk(this->force_link_entity_);
    force_lnk.EnableVelocityChecks(_ecm, true);
  }

  // Init ROS2 node + executor
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->ros_node_ = rclcpp::Node::make_shared("gz_planar_move");
  this->executor_ =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->executor_->add_node(this->ros_node_);
  this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });

  // TF broadcaster
  this->transform_broadcaster_ =
    std::make_shared<tf2_ros::TransformBroadcaster>(this->ros_node_);

  // Subscribe to cmd_vel
  this->vel_sub_ =
    this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      this->command_topic_, rclcpp::QoS(1),
      std::bind(&GazeboRosPlanarForceMove::cmdVelCallback,
                this, std::placeholders::_1));

  // Advertise odometry
  this->odometry_pub_ =
    this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      this->odometry_topic_, rclcpp::QoS(1));

  // Record initial pose
  auto init_pose = gz::sim::worldPose(this->canonical_link_entity_, _ecm);
  this->last_odom_pose_ = init_pose;
  this->fixed_x_   = init_pose.Pos().X();
  this->fixed_y_   = init_pose.Pos().Y();
  this->fixed_yaw_ = init_pose.Rot().Yaw();

  RCLCPP_INFO(this->ros_node_->get_logger(),
    "GazeboRosPlanarForceMove loaded (force_feedback=%d)", use_force_feedback_);
}

void GazeboRosPlanarForceMove::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) return;

  std::lock_guard<std::mutex> lock(this->lock_);

  gz::sim::Link canonical(this->canonical_link_entity_);

  auto pose_opt = canonical.WorldPose(_ecm);
  if (!pose_opt) return;
  const gz::math::Pose3d &pose = *pose_opt;

  if (this->use_force_feedback_) {
    gz::sim::Link force_lnk(this->force_link_entity_);
    auto lin_opt = force_lnk.WorldLinearVelocity(_ecm);
    auto ang_opt = force_lnk.WorldAngularVelocity(_ecm);
    if (!lin_opt || !ang_opt) return;

    // Rotate world velocity to body frame
    double yaw = pose.Rot().Yaw();
    double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
    double rlin_x =  cos_yaw * lin_opt->X() + sin_yaw * lin_opt->Y();
    double rlin_y = -sin_yaw * lin_opt->X() + cos_yaw * lin_opt->Y();
    double rang_z = ang_opt->Z();

    double f_x, f_y, t_rot;
    double v_sqr = this->x_ * this->x_ + this->y_ * this->y_ + this->rot_ * this->rot_;

    if (v_sqr <= this->v_dead_zone_) {
      if (this->base_cmd_vel_) {
        this->base_cmd_vel_ = false;
        this->fixed_x_   = pose.Pos().X();
        this->fixed_y_   = pose.Pos().Y();
        this->fixed_yaw_ = pose.Rot().Yaw();
      }
      // Position-hold force in body frame
      double wx = this->fixed_x_ - pose.Pos().X();
      double wy = this->fixed_y_ - pose.Pos().Y();
      f_x   =  cos_yaw * wx + sin_yaw * wy;
      f_y   = -sin_yaw * wx + cos_yaw * wy;
      t_rot = this->fixed_yaw_ - pose.Rot().Yaw();
      if (t_rot >  M_PI) t_rot -= 2 * M_PI;
      if (t_rot < -M_PI) t_rot += 2 * M_PI;

      f_x   *= (this->gain_x_   * 4);
      f_y   *= (this->gain_y_   * 4);
      t_rot *= (this->gain_rot_ * 4);
      f_x   += (this->x_   - rlin_x) * this->gain_x_   * 0.3;
      f_y   += (this->y_   - rlin_y) * this->gain_y_   * 0.3;
      t_rot += (this->rot_ - rang_z)  * this->gain_rot_ * 0.3;
    } else {
      this->base_cmd_vel_ = true;
      f_x   = (this->x_   - rlin_x) * this->gain_x_;
      f_y   = (this->y_   - rlin_y) * this->gain_y_;
      t_rot = (this->rot_ - rang_z)  * this->gain_rot_;
    }

    // Apply force in world frame (rotate back from body frame)
    double world_fx = cos_yaw * f_x - sin_yaw * f_y;
    double world_fy = sin_yaw * f_x + cos_yaw * f_y;
    force_lnk.AddWorldWrench(
      _ecm,
      gz::math::Vector3d(world_fx, world_fy, 0),
      gz::math::Vector3d(0, 0, t_rot));

  } else {
    // Direct velocity mode
    double yaw = pose.Rot().Yaw();
    double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
    gz::sim::Link link(this->canonical_link_entity_);
    link.SetLinearVelocity(_ecm,
      gz::math::Vector3d(
        this->x_ * cos_yaw - this->y_ * sin_yaw,
        this->y_ * cos_yaw + this->x_ * sin_yaw,
        0.0));
    link.SetAngularVelocity(_ecm,
      gz::math::Vector3d(0, 0, this->rot_));
  }

  // Odometry publishing
  if (this->odometry_rate_ > 0.0) {
    double secs_since =
      std::chrono::duration<double>(_info.simTime - this->last_odom_publish_time_).count();
    if (secs_since > 1.0 / this->odometry_rate_) {
      publishOdometry(_ecm, secs_since);
      this->last_odom_publish_time_ = _info.simTime;
    }
  }
}

void GazeboRosPlanarForceMove::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr cmd_msg)
{
  std::lock_guard<std::mutex> lock(this->lock_);
  this->x_   = cmd_msg->linear.x;
  this->y_   = this->enable_y_axis_ ? cmd_msg->linear.y : 0.0;
  this->rot_ = cmd_msg->angular.z;
}

void GazeboRosPlanarForceMove::publishOdometry(
  gz::sim::EntityComponentManager &_ecm,
  double step_time)
{
  gz::sim::Link canonical(this->canonical_link_entity_);
  auto pose_opt = canonical.WorldPose(_ecm);
  if (!pose_opt) return;
  const gz::math::Pose3d &pose = *pose_opt;

  rclcpp::Time current_time = this->ros_node_->now();

  // Publish TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = current_time;
  t.header.frame_id = this->odometry_frame_;
  t.child_frame_id  = this->robot_base_frame_;
  t.transform.translation.x = pose.Pos().X();
  t.transform.translation.y = pose.Pos().Y();
  t.transform.translation.z = pose.Pos().Z();
  t.transform.rotation.x = pose.Rot().X();
  t.transform.rotation.y = pose.Rot().Y();
  t.transform.rotation.z = pose.Rot().Z();
  t.transform.rotation.w = pose.Rot().W();
  this->transform_broadcaster_->sendTransform(t);

  // Build and publish odometry msg
  this->odom_.pose.pose.position.x = pose.Pos().X();
  this->odom_.pose.pose.position.y = pose.Pos().Y();
  this->odom_.pose.pose.orientation.x = pose.Rot().X();
  this->odom_.pose.pose.orientation.y = pose.Rot().Y();
  this->odom_.pose.pose.orientation.z = pose.Rot().Z();
  this->odom_.pose.pose.orientation.w = pose.Rot().W();
  this->odom_.pose.covariance[0]  = 0.00001;
  this->odom_.pose.covariance[7]  = 0.00001;
  this->odom_.pose.covariance[14] = 1000000000000.0;
  this->odom_.pose.covariance[21] = 1000000000000.0;
  this->odom_.pose.covariance[28] = 1000000000000.0;
  this->odom_.pose.covariance[35] = 0.001;

  double lin_x = (pose.Pos().X() - this->last_odom_pose_.Pos().X()) / step_time;
  double lin_y = (pose.Pos().Y() - this->last_odom_pose_.Pos().Y()) / step_time;

  float last_yaw    = static_cast<float>(this->last_odom_pose_.Rot().Yaw());
  float current_yaw = static_cast<float>(pose.Rot().Yaw());
  while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
  while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
  this->odom_.twist.twist.angular.z = (current_yaw - last_yaw) / step_time;

  this->last_odom_pose_ = pose;

  float yaw = static_cast<float>(pose.Rot().Yaw());
  this->odom_.twist.twist.linear.x =
     std::cos(yaw) * lin_x + std::sin(yaw) * lin_y;
  this->odom_.twist.twist.linear.y =
     std::cos(yaw) * lin_y - std::sin(yaw) * lin_x;

  this->odom_.header.stamp       = current_time;
  this->odom_.header.frame_id    = this->odometry_frame_;
  this->odom_.child_frame_id     = this->robot_base_frame_;

  this->odometry_pub_->publish(this->odom_);
}

}  // namespace gazebo
