#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <thread>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Types.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{

class GazeboRosPlanarForceMove :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  GazeboRosPlanarForceMove();
  ~GazeboRosPlanarForceMove() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  void publishOdometry(
    gz::sim::EntityComponentManager &_ecm,
    double step_time);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity canonical_link_entity_{gz::sim::kNullEntity};

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  nav_msgs::msg::Odometry odom_;

  std::mutex lock_;

  std::string command_topic_{"cmd_vel"};
  std::string odometry_topic_{"odom"};
  std::string odometry_frame_{"odom"};
  std::string robot_base_frame_{"base_link"};
  double odometry_rate_{20.0};
  bool enable_y_axis_{true};

  double x_{0.0};
  double y_{0.0};
  double rot_{0.0};

  bool use_force_feedback_{false};
  gz::sim::Entity force_link_entity_{gz::sim::kNullEntity};
  double gain_x_{20000};
  double gain_y_{20000};
  double gain_rot_{5000};
  double v_dead_zone_{0.0001};
  bool base_cmd_vel_{true};
  double fixed_x_{0.0};
  double fixed_y_{0.0};
  double fixed_yaw_{0.0};

  std::chrono::steady_clock::duration last_odom_publish_time_{};
  gz::math::Pose3d last_odom_pose_;
};

}  // namespace gazebo
