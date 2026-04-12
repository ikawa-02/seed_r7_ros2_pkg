#include <regex>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "seed_r7_ros_controller/msg/robot_info.hpp"

class RecordRobotInfo : public rclcpp::Node
{
public:
  RecordRobotInfo()
  : Node("record_robot_info_node")
  {
    file_path_ = std::string(std::getenv("HOME")) + "/Desktop/Record_Data.csv";

    std::ofstream ofs(file_path_, std::ios_base::trunc);

    // BOM for UTF-8 (Excel compatibility)
    unsigned char bom[] = {0xEF, 0xBB, 0xBF};
    ofs.write(reinterpret_cast<char *>(bom), sizeof(bom));

    start_time_ = now();
    ofs << "経過時間,バッテリ,";
    ofs << "指令速度(x), 指令速度(y), 指令速度(theta), ";
    ofs << "実速度(x), 実速度(y), 実速度(theta), ";
    ofs << "実位置(x), 実位置(y), 実位置(theta), ";
    ofs << "位置(右前),位置(右後),位置(左前),位置(左後),";
    ofs << "電流(右前),電流(右後),電流(左前),電流(左後),";
    ofs << "温度(右前),温度(右後),温度(左前),温度(左後),";
    ofs << "状態(右前),状態(右後),状態(左前),状態(左後),状態(全体)";
    ofs << std::endl;

    sub_ = create_subscription<seed_r7_ros_controller::msg::RobotInfo>(
      "/seed_r7_ros_controller/robot_info", 1,
      std::bind(&RecordRobotInfo::GetRobotInfo, this, std::placeholders::_1));
  }

private:
  void GetRobotInfo(const seed_r7_ros_controller::msg::RobotInfo::SharedPtr data)
  {
    std::ofstream ofs(file_path_, std::ios_base::app);

    double elapsed = (now() - start_time_).seconds();
    ofs << elapsed << "," << data->robot.voltage << ",";
    ofs << data->robot.cmd_vel.x << "," << data->robot.cmd_vel.y << ","
        << data->robot.cmd_vel.theta << ",";
    ofs << data->robot.odom_vel.x << "," << data->robot.odom_vel.y << ","
        << data->robot.odom_vel.theta << ",";
    ofs << data->robot.odom_pos.x << "," << data->robot.odom_pos.y << ","
        << data->robot.odom_pos.theta << ",";
    ofs << data->driver[0].position << "," << data->driver[1].position << ","
        << data->driver[2].position << "," << data->driver[3].position << ",";
    ofs << data->driver[0].current << "," << data->driver[1].current << ","
        << data->driver[2].current << "," << data->driver[3].current << ",";
    ofs << data->driver[0].temp << "," << data->driver[1].temp << ","
        << data->driver[2].temp << "," << data->driver[3].temp << ",";
    ofs << data->driver[0].status << "," << data->driver[1].status << ","
        << data->driver[2].status << "," << data->driver[3].status << ",";
    ofs << data->robot.status;
    ofs << std::endl;

    RCLCPP_INFO(get_logger(), "record robot info");
  }

  rclcpp::Subscription<seed_r7_ros_controller::msg::RobotInfo>::SharedPtr sub_;
  std::string file_path_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RecordRobotInfo>());
  rclcpp::shutdown();
  return 0;
}
