#include <thread>
#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"
#include "seed_r7_ros_controller/seed_r7_mover_controller.h"
#include "seed_r7_ros_controller/seed_r7_hand_controller.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_hardware
{

RobotHW::RobotHW()
: converter_loader_("seed_r7_ros_controller", "seed_converter::StrokeConverter"),
  executor_running_(false)
{
}

RobotHW::~RobotHW()
{
  if (executor_running_) {
    executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_running_ = false;
  }
}

hardware_interface::CallbackReturn RobotHW::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create internal node — load hw_settings.yaml if provided via URDF <param>
  rclcpp::NodeOptions hw_node_options;
  if (info_.hardware_parameters.count("joint_settings_file")) {
    const std::string & yaml_file = info_.hardware_parameters.at("joint_settings_file");
    if (!yaml_file.empty()) {
      hw_node_options.arguments({"--ros-args", "--params-file", yaml_file});
      hw_node_options.automatically_declare_parameters_from_overrides(true);
    }
  }
  hw_node_ = std::make_shared<rclcpp::Node>("seed_r7_hardware_interface", hw_node_options);

  // Prefer to add our node to the controller_manager's executor if available
  if (auto exec = params.executor.lock()) {
    exec->add_node(hw_node_);
    executor_running_ = false;  // managed externally
  } else {
    // Fallback: spin in our own thread
    executor_.add_node(hw_node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });
    executor_running_ = true;
  }

  // ---- Read hardware parameters (from URDF <hardware><param> or node params) ----
  std::string port_upper = "/dev/aero_upper";
  std::string port_lower = "/dev/aero_lower";
  robot_model_plugin_ = "seed_r7_robot_interface/TypeF";

  if (info_.hardware_parameters.count("port_upper")) {
    port_upper = info_.hardware_parameters.at("port_upper");
  }
  if (info_.hardware_parameters.count("port_lower")) {
    port_lower = info_.hardware_parameters.at("port_lower");
  }
  if (info_.hardware_parameters.count("robot_model_plugin")) {
    robot_model_plugin_ = info_.hardware_parameters.at("robot_model_plugin");
  }

  // Controller rate
  CONTROL_PERIOD_US_ = 50 * 1000;  // default 50ms
  OVERLAP_SCALE_ = 2.8f;
  if (info_.hardware_parameters.count("controller_rate")) {
    double rate = std::stod(info_.hardware_parameters.at("controller_rate"));
    CONTROL_PERIOD_US_ = static_cast<int>((1000.0 * 1000.0) / rate);
  }
  if (info_.hardware_parameters.count("overlap_scale")) {
    OVERLAP_SCALE_ = static_cast<float>(std::stod(info_.hardware_parameters.at("overlap_scale")));
  }

  wheel_vel_limit_ = 9.0f;
  pub_robot_info_ = false;
  if (info_.hardware_parameters.count("wheel_vel_limit")) {
    wheel_vel_limit_ = static_cast<float>(std::stod(info_.hardware_parameters.at("wheel_vel_limit")));
  }
  if (info_.hardware_parameters.count("pub_robot_info")) {
    pub_robot_info_ = (info_.hardware_parameters.at("pub_robot_info") == "true");
  }

  // csv_config_dir for stroke converter
  if (info_.hardware_parameters.count("csv_config_dir")) {
    hw_node_->declare_parameter("csv_config_dir", info_.hardware_parameters.at("csv_config_dir"));
  } else {
    hw_node_->declare_parameter<std::string>("csv_config_dir", "");
  }

  RCLCPP_INFO(hw_node_->get_logger(), "upper_port: %s", port_upper.c_str());
  RCLCPP_INFO(hw_node_->get_logger(), "lower_port: %s", port_lower.c_str());
  RCLCPP_INFO(hw_node_->get_logger(), "cycle: %f [ms], overlap_scale %f",
              CONTROL_PERIOD_US_ * 0.001, OVERLAP_SCALE_);

  // ---- Create upper/lower controllers (they read joint_settings from hw_node_) ----
  controller_upper_ = std::make_shared<robot_hardware::UpperController>(port_upper, hw_node_);
  controller_lower_ = std::make_shared<robot_hardware::LowerController>(port_lower, hw_node_);

  upper_connected_ = controller_upper_->is_open_;
  lower_connected_ = controller_lower_->is_open_;

  // ---- Get joint names from node parameters ----
  joint_names_upper_ = controller_upper_->name_;
  joint_names_lower_ = controller_lower_->name_;

  number_of_angles_ = joint_names_upper_.size() + joint_names_lower_.size();

  joint_list_.resize(number_of_angles_);
  for (size_t i = 0; i < joint_names_upper_.size(); ++i) {
    joint_list_[i] = joint_names_upper_[i];
  }
  for (size_t i = 0; i < joint_names_lower_.size(); ++i) {
    joint_list_[joint_names_upper_.size() + i] = joint_names_lower_[i];
  }

  // ---- Load stroke converter plugin ----
  try {
    stroke_converter_ = converter_loader_.createSharedInstance(robot_model_plugin_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(hw_node_->get_logger(), "Failed to load stroke converter plugin: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!stroke_converter_->initialize(hw_node_)) {
    RCLCPP_ERROR(hw_node_->get_logger(), "Failed to initialize stroke converter");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- Initialize state arrays ----
  joint_types_.resize(number_of_angles_, JointType::ROTATIONAL);
  joint_control_methods_.resize(number_of_angles_, ControlMethod::POSITION);
  joint_position_.resize(number_of_angles_, 0.0);
  joint_velocity_.resize(number_of_angles_, 0.0);
  joint_effort_.resize(number_of_angles_, 0.0);
  joint_position_command_.resize(number_of_angles_, 0.0);

  wheel_angles_.resize(4, 0.0);
  wheel_velocities_.resize(4, 0.0);
  prev_ref_strokes_.resize(number_of_angles_, 0.0);
  initialized_flag_ = false;

  // ---- Get firmware versions ----
  std::string upper_firmware = controller_upper_->getFirmwareVersion();
  std::string lower_firmware = controller_lower_->getFirmwareVersion();
  RCLCPP_INFO(hw_node_->get_logger(), "Upper Firmware Ver. is [ %s ]", upper_firmware.c_str());
  RCLCPP_INFO(hw_node_->get_logger(), "Lower Firmware Ver. is [ %s ]", lower_firmware.c_str());

  // ---- Battery voltage publisher ----
  bat_vol_pub_ = hw_node_->create_publisher<std_msgs::msg::Float32>("voltage", 1);
  bat_vol_timer_ = hw_node_->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RobotHW::getBatteryVoltage, this));

  // ---- Robot info publisher ----
  if (pub_robot_info_) {
    robot_info_.robot.firmware = lower_firmware;
    robot_info_pub_ = hw_node_->create_publisher<seed_r7_ros_controller::msg::RobotInfo>("robot_info", 100);
    robot_info_timer_ = hw_node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotHW::pubRobotInfo, this));
  }

  // ---- cmd_vel / odom subscriptions ----
  cmd_vel_sub_ = hw_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    std::bind(&RobotHW::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = hw_node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1,
    std::bind(&RobotHW::odomCallback, this, std::placeholders::_1));

  // ---- reset_robot_status service ----
  reset_robot_status_server_ = hw_node_->create_service<seed_r7_ros_controller::srv::ResetRobotStatus>(
    "reset_robot_status",
    std::bind(&RobotHW::resetRobotStatusCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // ---- Diagnostics ----
  pre_diag_level_ = 0;
  pre_diag_msg_ = "";
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(hw_node_);
  diagnostic_updater_->setHardwareID("SEED-Noid-Mover");
  diagnostic_updater_->add("RobotStatus", this, &RobotHW::setDiagnostics);

  // ---- MoverController and HandController ----
  mover_controller_ = std::make_shared<MoverController>(hw_node_, this);
  hand_controller_ = std::make_shared<HandController>(hw_node_, this);

  robot_status_.p_stopped_err_ = false;

  RCLCPP_INFO(hw_node_->get_logger(), "RobotHW::on_init() completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < number_of_angles_; ++i) {
    state_interfaces.emplace_back(
      joint_list_[i], hardware_interface::HW_IF_POSITION, &joint_position_[i]);
    state_interfaces.emplace_back(
      joint_list_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < number_of_angles_; ++i) {
    command_interfaces.emplace_back(
      joint_list_[i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RobotHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Initial position read
  readPos(hw_node_->now(), rclcpp::Duration(0, 0), true);

  // Initialize command to current position
  for (unsigned int j = 0; j < number_of_angles_; ++j) {
    joint_position_command_[j] = joint_position_[j];
  }
  initialized_flag_ = true;

  RCLCPP_INFO(hw_node_->get_logger(), "RobotHW activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(hw_node_->get_logger(), "RobotHW deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RobotHW::readPos(const rclcpp::Time & /*time*/, const rclcpp::Duration & period, bool update)
{
  mutex_lower_.lock();
  mutex_upper_.lock();
  if (update) {
    std::thread t1([&]() { controller_upper_->getPosition(); });
    std::thread t2([&]() { controller_lower_->getPosition(); });
    t1.join();
    t2.join();
  }
  mutex_upper_.unlock();
  mutex_lower_.unlock();

  // whole body strokes
  std::vector<int16_t> act_strokes;
  std::vector<int16_t> act_upper_strokes;
  std::vector<int16_t> act_lower_strokes;

  controller_upper_->remapAeroToRos(act_upper_strokes, controller_upper_->raw_data_);
  controller_lower_->remapAeroToRos(act_lower_strokes, controller_lower_->raw_data_);

  act_strokes.insert(act_strokes.end(), act_upper_strokes.begin(), act_upper_strokes.end());
  act_strokes.insert(act_strokes.end(), act_lower_strokes.begin(), act_lower_strokes.end());

  // convert from stroke to angle
  std::vector<double> act_positions(number_of_angles_);
  stroke_converter_->Stroke2Angle(act_positions, act_strokes);

  double dt = period.seconds();
  (void)dt;
  for (unsigned int j = 0; j < number_of_angles_; j++) {
    joint_position_[j] = act_positions[j];
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 0.0;
  }

  if (!initialized_flag_) {
    for (unsigned int j = 0; j < number_of_angles_; j++) {
      joint_position_command_[j] = joint_position_[j];
    }
    initialized_flag_ = true;
  }

  // check robot status
  setRobotStatus();

  if (robot_status_.p_stopped_err_) {
    RCLCPP_WARN(hw_node_->get_logger(), "The robot is protective stopped, please release it.");
  }
  if (robot_status_.connection_err_ && robot_status_.calib_err_) {
    RCLCPP_WARN(hw_node_->get_logger(), "The robot is Emergency stopped, please release it.");
  }

  // update wheel velocities
  for (size_t i = 0; i < wheel_angles_.size(); i++) {
    double period_sec = period.seconds();
    if (period_sec > 0.0 &&
        (std::abs(controller_lower_->wheel_angles_.at(i) - wheel_angles_.at(i)) / period_sec) < wheel_vel_limit_) {
      wheel_velocities_.at(i) =
        (controller_lower_->wheel_angles_.at(i) - wheel_angles_.at(i)) / period_sec;
    }
  }

  // noise filter
  int count_zero_vel = std::count(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);
  if (count_zero_vel == 3) fill(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);

  wheel_angles_ = controller_lower_->wheel_angles_;
}

hardware_interface::return_type RobotHW::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // In ROS2 ros2_control, read() is called each control cycle.
  // The actual reading happens in readPos (called from write to avoid double-reading).
  (void)time;
  (void)period;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHW::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::vector<double> ref_positions(number_of_angles_);
  for (unsigned int j = 0; j < number_of_angles_; ++j) {
    ref_positions[j] = joint_position_command_[j];
  }

  std::vector<bool> mask_positions(number_of_angles_, true);

  // convert from angle to stroke
  std::vector<int16_t> ref_strokes(ref_positions.size());
  stroke_converter_->Angle2Stroke(ref_strokes, ref_positions);

  for (size_t i = 0; i < number_of_angles_; ++i) {
    double tmp = ref_strokes[i];
    if (tmp == prev_ref_strokes_[i]) {
      mask_positions[i] = false;
    }
    prev_ref_strokes_[i] = tmp;
  }

  // masking
  std::vector<int16_t> snt_strokes(ref_strokes);
  for (size_t i = 0; i < ref_strokes.size(); ++i) {
    if (!mask_positions[i]) snt_strokes[i] = 0x7FFF;
  }

  // split strokes into upper and lower
  std::vector<int16_t> upper_strokes;
  std::vector<int16_t> lower_strokes;

  if (controller_upper_->is_open_) controller_upper_->remapRosToAero(upper_strokes, snt_strokes);
  else controller_upper_->remapRosToAero(upper_strokes, ref_strokes);
  if (controller_lower_->is_open_) controller_lower_->remapRosToAero(lower_strokes, snt_strokes);
  else controller_lower_->remapRosToAero(lower_strokes, ref_strokes);

  uint16_t time_csec = static_cast<uint16_t>((OVERLAP_SCALE_ * CONTROL_PERIOD_US_) / (1000 * 10));

  mutex_lower_.lock();
  mutex_upper_.lock();
  {
    std::thread t1([&]() { controller_upper_->sendPosition(time_csec, upper_strokes); });
    std::thread t2([&]() { controller_lower_->sendPosition(time_csec, lower_strokes); });
    t1.join();
    t2.join();
  }
  mutex_upper_.unlock();
  mutex_lower_.unlock();

  // read back positions
  readPos(time, period, false);

  return hardware_interface::return_type::OK;
}

//////////////////////////////////////////////
// Specific functions
//////////////////////////////////////////////

void RobotHW::runHandScript(uint8_t _number, uint16_t _script, uint8_t _current)
{
  mutex_upper_.lock();
  if (_script == 2) {
    controller_upper_->runScript(_number, 4);
    usleep(20 * 1000);
    controller_upper_->setCurrent(_number, _current, _current);
  }
  controller_upper_->runScript(_number, _script);
  RCLCPP_INFO(hw_node_->get_logger(), "sendnum : %d, script : %d", _number, _script);
  mutex_upper_.unlock();
}

void RobotHW::turnWheel(std::vector<int16_t> & _vel)
{
  mutex_lower_.lock();
  controller_lower_->sendVelocity(_vel);
  mutex_lower_.unlock();
}

void RobotHW::onWheelServo(bool _value)
{
  mutex_lower_.lock();
  controller_lower_->onServo(_value);
  mutex_lower_.unlock();
}

void RobotHW::getBatteryVoltage()
{
  std_msgs::msg::Float32 voltage;
  mutex_lower_.lock();
  std::vector<uint16_t> data = controller_lower_->getBatteryVoltage();
  mutex_lower_.unlock();
  voltage.data = data.at(30) / 10.0f;
  bat_vol_pub_->publish(voltage);

  robot_info_.robot.voltage = data.at(30) / 10.0;
  robot_info_.driver[0].temp = static_cast<uint8_t>(data.at(2) >> 8);
  robot_info_.driver[1].temp = static_cast<uint8_t>(data.at(3) >> 8);
  robot_info_.driver[2].temp = static_cast<uint8_t>(data.at(4) >> 8);
  robot_info_.driver[3].temp = static_cast<uint8_t>(data.at(5) >> 8);
}

void RobotHW::pubRobotInfo()
{
  std::vector<uint16_t> current;
  current.resize(31, 0);

  mutex_lower_.lock();
  if (!robot_status_.connection_err_ && !robot_status_.calib_err_) {
    current = controller_lower_->getMotorCurrent(0);
  }
  mutex_lower_.unlock();

  robot_info_.driver[0].current = current.at(2);
  robot_info_.driver[1].current = current.at(3);
  robot_info_.driver[2].current = current.at(4);
  robot_info_.driver[3].current = current.at(5);

  robot_info_.driver[0].position =
    static_cast<int16_t>(controller_lower_->wheel_angles_.at(0) * (180 / M_PI));
  robot_info_.driver[1].position =
    static_cast<int16_t>(controller_lower_->wheel_angles_.at(1) * (180 / M_PI));
  robot_info_.driver[2].position =
    static_cast<int16_t>(controller_lower_->wheel_angles_.at(2) * (180 / M_PI));
  robot_info_.driver[3].position =
    static_cast<int16_t>(controller_lower_->wheel_angles_.at(3) * (180 / M_PI));

  robot_info_.header.stamp = hw_node_->now();
  robot_info_pub_->publish(robot_info_);
}

void RobotHW::runLedScript(uint8_t _number, uint16_t _script)
{
  mutex_lower_.lock();
  controller_lower_->runScript(_number, _script);
  mutex_lower_.unlock();
}

void RobotHW::setRobotStatus()
{
  comm_err_ = controller_lower_->comm_err_ || controller_upper_->comm_err_;

  robot_status_.connection_err_ = controller_lower_->robot_status_.connection_err_ ||
    controller_upper_->robot_status_.connection_err_;
  robot_status_.calib_err_ = controller_lower_->robot_status_.calib_err_ ||
    controller_upper_->robot_status_.calib_err_;
  robot_status_.motor_err_ = controller_lower_->robot_status_.motor_err_ ||
    controller_upper_->robot_status_.motor_err_;
  robot_status_.temp_err_ = controller_lower_->robot_status_.temp_err_ ||
    controller_upper_->robot_status_.temp_err_;
  robot_status_.res_err_ = controller_lower_->robot_status_.res_err_ ||
    controller_upper_->robot_status_.res_err_;
  robot_status_.step_out_err_ = controller_lower_->robot_status_.step_out_err_ ||
    controller_upper_->robot_status_.step_out_err_;
  robot_status_.p_stopped_err_ = controller_lower_->robot_status_.p_stopped_err_ ||
    controller_upper_->robot_status_.p_stopped_err_;
  robot_status_.power_err_ = controller_lower_->robot_status_.power_err_ ||
    controller_upper_->robot_status_.power_err_;

  diagnostic_updater_->force_update();
}

void RobotHW::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (comm_err_) {
    stat.summary(2, "Now calibrating, or the USB cable is plugged out");
  } else if (robot_status_.power_err_) {
    stat.summary(2, "Power failed, please check the battery");
  } else if (robot_status_.connection_err_) {
    stat.summary(2, "Connection error occurred, please check the cable");
  } else if (robot_status_.temp_err_) {
    stat.summary(2, "Motor driver is high temperature, please reboot the robot");
  } else if (robot_status_.p_stopped_err_) {
    stat.summary(2, "Protective stopped, please release it");
  } else if (robot_status_.calib_err_) {
    stat.summary(2, "Calibration error occurred, please recalibration");
  } else if (robot_status_.step_out_err_) {
    stat.summary(1, "Step-out has occurred");
  } else if (robot_status_.res_err_) {
    stat.summary(1, "Response error has occurred");
  } else {
    stat.summary(0, "System all green");
  }

  if (robot_status_.connection_err_ && robot_status_.calib_err_) {
    stat.summary(2, "E-Stop switch is pushed, please release it");
  }

  stat.add("Communication Error", comm_err_);
  stat.add("Emergency Stopped", robot_status_.connection_err_ && robot_status_.calib_err_);
  stat.add("Protective Stopped", robot_status_.p_stopped_err_);
  stat.add("Connection Error", robot_status_.connection_err_);
  stat.add("Calibration Error", robot_status_.calib_err_);
  stat.add("Motor Servo OFF", robot_status_.motor_err_);
  stat.add("Temperature Error", robot_status_.temp_err_);
  stat.add("Response Error", robot_status_.res_err_);
  stat.add("Step Out Occurred", robot_status_.step_out_err_);
  stat.add("Power Failed", robot_status_.power_err_);

  robot_info_.robot.level = stat.level;
  robot_info_.robot.status = stat.message;

  if (stat.level == 0 || stat.level == 1) {
    for (int i = 0; i < 4; ++i) {
      robot_info_.driver[i].flag = 0;
      robot_info_.driver[i].status = "Normal";
    }
  } else {
    if (stat.level != pre_diag_level_ || stat.message != pre_diag_msg_) {
      mutex_lower_.lock();
      std::vector<uint16_t> data = controller_lower_->getRobotStatus(0);
      mutex_lower_.unlock();
      for (int i = 0; i < 4; ++i) {
        robot_info_.driver[i].flag = data.at(i + 2);
        if (data.at(i + 2) >> 0 & 1) robot_info_.driver[i].status = "connection error";
        else if (data.at(i + 2) >> 1 & 1) robot_info_.driver[i].status = "calibration error";
        else if (data.at(i + 2) >> 3 & 1) robot_info_.driver[i].status = "temperature error";
        else if (data.at(i + 2) >> 6 & 1) robot_info_.driver[i].status = "protective stopped";
        else if (data.at(i + 2) >> 7 & 1) robot_info_.driver[i].status = "power error";
        else if (data.at(i + 2) >> 2 & 1) robot_info_.driver[i].status = "servo off";
        else if (data.at(i + 2) >> 4 & 1) robot_info_.driver[i].status = "response error";
        else if (data.at(i + 2) >> 5 & 1) robot_info_.driver[i].status = "step out error";
      }
    }
  }

  pre_diag_level_ = stat.level;
  pre_diag_msg_ = stat.message;
}

void RobotHW::resetRobotStatusCallback(
  const seed_r7_ros_controller::srv::ResetRobotStatus::Request::SharedPtr /*_req*/,
  seed_r7_ros_controller::srv::ResetRobotStatus::Response::SharedPtr _res)
{
  RCLCPP_WARN(hw_node_->get_logger(), "reset robot status");
  mutex_lower_.lock();
  controller_lower_->getRobotStatus(static_cast<int8_t>(0xFF));
  mutex_lower_.unlock();
  _res->result = "reset status succeeded";
}

void RobotHW::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr _cmd_vel)
{
  robot_info_.robot.cmd_vel.x = _cmd_vel->linear.x;
  robot_info_.robot.cmd_vel.y = _cmd_vel->linear.y;
  robot_info_.robot.cmd_vel.theta = _cmd_vel->angular.z;
}

void RobotHW::odomCallback(const nav_msgs::msg::Odometry::SharedPtr _odom)
{
  robot_info_.robot.odom_vel.x = _odom->twist.twist.linear.x;
  robot_info_.robot.odom_vel.y = _odom->twist.twist.linear.y;
  robot_info_.robot.odom_vel.theta = _odom->twist.twist.angular.z;

  robot_info_.robot.odom_pos.x = _odom->pose.pose.position.x;
  robot_info_.robot.odom_pos.y = _odom->pose.pose.position.y;
  robot_info_.robot.odom_pos.theta = tf2::getYaw(_odom->pose.pose.orientation);
}

}  // namespace robot_hardware

// pluginlib export
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotHW, hardware_interface::SystemInterface)
