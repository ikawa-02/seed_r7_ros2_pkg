#include "seed_r7_ros_controller/seed_r7_upper_controller.h"


robot_hardware::UpperController::UpperController(const std::string& _port,
                                                  rclcpp::Node::SharedPtr node)
  : logger_(node->get_logger())
{
  // Get joint settings from node parameters
  if (!node->has_parameter("joint_settings.upper.joints")) {
    node->declare_parameter("joint_settings.upper.joints", std::vector<std::string>{});
  }
  if (!node->has_parameter("joint_settings.upper.aero_index")) {
    node->declare_parameter("joint_settings.upper.aero_index", std::vector<int64_t>{});
  }
  if (!node->has_parameter("joint_settings.raw_data_size")) {
    node->declare_parameter("joint_settings.raw_data_size", 0);
  }
  if (!node->has_parameter("joint_settings.body_data_size")) {
    node->declare_parameter("joint_settings.body_data_size", 0);
  }

  name_ = node->get_parameter("joint_settings.upper.joints").as_string_array();

  auto aero_index_64 = node->get_parameter("joint_settings.upper.aero_index").as_integer_array();
  aero_index_.assign(aero_index_64.begin(), aero_index_64.end());

  int raw_data_size = static_cast<int>(node->get_parameter("joint_settings.raw_data_size").as_int());
  int body_data_size = static_cast<int>(node->get_parameter("joint_settings.body_data_size").as_int());

  upper_ = new aero::controller::AeroCommand();
  if (upper_->openPort(_port, BAUDRATE)) {
    RCLCPP_INFO(logger_, "%s is connected", _port.c_str());
    upper_->flushPort();
    is_open_ = true;
  } else {
    RCLCPP_ERROR(logger_, "%s is not connected", _port.c_str());
    is_open_ = false;
  }

  raw_data_.resize(raw_data_size);
  fill(raw_data_.begin(), raw_data_.end(), 0);

  // make table for remap aero <-> ros
  aero_table_.resize(body_data_size);
  for (size_t i = 0; i < aero_table_.size(); ++i) {
    size_t index = std::distance(aero_index_.begin(),
                                 std::find(aero_index_.begin(), aero_index_.end(), static_cast<int>(i)));
    if (index != aero_index_.size()) aero_table_.at(i) = std::make_pair(index, name_.at(index));
  }
}

robot_hardware::UpperController::~UpperController()
{
  if (is_open_) upper_->closePort();
  delete upper_;
}

void robot_hardware::UpperController::getPosition()
{
  if (is_open_) raw_data_ = upper_->getPosition(0);
  checkRobotStatus();
}

void robot_hardware::UpperController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
  if (is_open_) raw_data_ = upper_->actuateByPosition(_time, _data.data());
  else raw_data_.assign(_data.begin(), _data.end());
  checkRobotStatus();
}

void robot_hardware::UpperController::remapAeroToRos
(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero)
{
  _ros.resize(name_.size());
  for (size_t i = 0; i < _ros.size(); ++i) {
    if (aero_index_.at(i) != -1) _ros.at(i) = _aero.at(aero_index_.at(i));
  }
}

void robot_hardware::UpperController::remapRosToAero
(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros)
{
  _aero.resize(aero_table_.size());
  for (size_t i = 0; i < _aero.size(); ++i) {
    _aero.at(i) = _ros.at(aero_table_.at(i).first);
  }
}

void robot_hardware::UpperController::setCurrent(uint8_t _number, uint8_t _max, uint8_t _down)
{
  if (is_open_) upper_->setCurrent(_number, _max, _down);
}

void robot_hardware::UpperController::runScript(uint8_t _number, uint16_t _script)
{
  if (is_open_) upper_->runScript(_number, _script);
}

std::string robot_hardware::UpperController::getFirmwareVersion()
{
  if (is_open_) return upper_->getVersion(0);
  else return "";
}

void robot_hardware::UpperController::checkRobotStatus()
{
  if (is_open_) comm_err_ = upper_->comm_err_;
  else comm_err_ = false;

  int16_t status_bit = raw_data_.back();

  if (!is_open_) status_bit = 0;
  robot_status_.connection_err_ = (status_bit >> error_bit_t::can1_connection) & 1 ||
    (status_bit >> error_bit_t::can2_connection) & 1;
  robot_status_.calib_err_ = (status_bit >> error_bit_t::can1_calibration) & 1 ||
    (status_bit >> error_bit_t::can2_calibration) & 1;
  robot_status_.motor_err_ = (status_bit >> error_bit_t::can1_motor_status) & 1 ||
    (status_bit >> error_bit_t::can2_motor_status) & 1;
  robot_status_.temp_err_ = (status_bit >> error_bit_t::can1_temperature) & 1 ||
    (status_bit >> error_bit_t::can2_temperature) & 1;
  robot_status_.res_err_ = (status_bit >> error_bit_t::can1_response) & 1 ||
    (status_bit >> error_bit_t::can2_response) & 1;
  robot_status_.step_out_err_ = (status_bit >> error_bit_t::can1_step_out) & 1 ||
    (status_bit >> error_bit_t::can2_step_out) & 1;
  robot_status_.p_stopped_err_ = (status_bit >> error_bit_t::can1_protective_stopped) & 1 ||
    (status_bit >> error_bit_t::can2_protective_stopped) & 1;
  robot_status_.power_err_ = (status_bit >> error_bit_t::can1_power) & 1 ||
    (status_bit >> error_bit_t::can2_power) & 1;
}
