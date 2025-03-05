#include "ros2_hw_pi_hat/ros2_hw_pi_hat.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hw_pi_hat_lib/hw_pi_hat_lib.hpp"
#include "hw_pi_hat_msgs/msg/servos.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

Ros2HwPiHat::Ros2HwPiHat() : Node("ros2_hw_pi_hat_interface"), board() {
  std::vector<uint8_t> expected_servos = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  board.setRecieve(true);
  sleep(1);

  // check for servos available
  for (std::vector<uint8_t>::iterator it = expected_servos.begin();
       it != expected_servos.end(); it++) {
    std::optional<uint8_t> id = board.getServoId(*it);
    if (id) {
      servos.push_back(id.value());
    }
  }
  RCLCPP_INFO(this->get_logger(), "Found '%lu'", servos.size());

  battery_pub = this->create_publisher<std_msgs::msg::UInt16>("battery", 10);
  battery_timer =
      this->create_wall_timer(150ms, std::bind(&Ros2HwPiHat::battery_cb, this));

  temp_pub =
      this->create_publisher<std_msgs::msg::UInt8MultiArray>("temperature", 10);
  temp_timer =
      this->create_wall_timer(50ms, std::bind(&Ros2HwPiHat::temp_cb, this));

  pos_pub =
      this->create_publisher<std_msgs::msg::Int16MultiArray>("position", 10);
  pos_timer =
      this->create_wall_timer(50ms, std::bind(&Ros2HwPiHat::pos_cb, this));

  vin_pub = this->create_publisher<std_msgs::msg::UInt16MultiArray>("vin", 10);
  vin_timer =
      this->create_wall_timer(50ms, std::bind(&Ros2HwPiHat::vin_cb, this));

  stop_subscriber = this->create_subscription<std_msgs::msg::Empty>(
      "stop", 10,
      std::bind(&Ros2HwPiHat::stop_cb, this, std::placeholders::_1));

  servo_subscriber = this->create_subscription<hw_pi_hat_msgs::msg::Servos>(
      "set_servos_pos", 10,
      std::bind(&Ros2HwPiHat::servo_cb, this, std::placeholders::_1));
}

Ros2HwPiHat::~Ros2HwPiHat() {}

void Ros2HwPiHat::battery_cb() {
  auto msg = std_msgs::msg::UInt16();
  std::optional<uint16_t> battery = board.getBattery();
  if (battery) {
    msg.data = battery.value();
  } else {
    msg.data = 0.;
  }
  battery_pub->publish(msg);
}

void Ros2HwPiHat::temp_cb() {
  std::vector<uint8_t> temps;
  // Retrive temperatures
  for (std::vector<uint8_t>::iterator it = servos.begin(); it != servos.end();
       it++) {
    std::optional<uint8_t> temp = board.getServoTemp(*it);
    if (temp)
      temps.push_back(temp.value());
  }
  if (temps.size() != servos.size()) {
    RCLCPP_INFO(this->get_logger(), "Could not retrive all the temperatures");
    return;
  }
  auto msg = std_msgs::msg::UInt8MultiArray();
  msg.layout.dim.resize(1);
  msg.layout.dim[0].size = temps.size();
  msg.layout.dim[0].label = "temperatures";
  msg.layout.dim[0].stride = 1;

  msg.data = temps;
  temp_pub->publish(msg);
}

void Ros2HwPiHat::pos_cb() {
  std::vector<int16_t> positions;
  // Retrive temperatures
  for (std::vector<uint8_t>::iterator it = servos.begin(); it != servos.end();
       it++) {
    std::optional<int16_t> pos = board.getServoPos(*it);
    if (pos)
      positions.push_back(pos.value());
  }
  if (positions.size() != servos.size()) {
    RCLCPP_INFO(this->get_logger(), "Could not retrive all the positions");
    return;
  }
  auto msg = std_msgs::msg::Int16MultiArray();
  msg.layout.dim.resize(1);
  msg.layout.dim[0].size = positions.size();
  msg.layout.dim[0].label = "positions";
  msg.layout.dim[0].stride = 1;

  msg.data = positions;
  pos_pub->publish(msg);
}

void Ros2HwPiHat::vin_cb() {
  std::vector<uint16_t> vins;
  // Retrive temperatures
  for (std::vector<uint8_t>::iterator it = servos.begin(); it != servos.end();
       it++) {
    std::optional<uint16_t> vin = board.getServoVin(*it);
    if (vin)
      vins.push_back(vin.value());
  }
  if (vins.size() != servos.size()) {
    RCLCPP_INFO(this->get_logger(), "Could not retrive all the voltage inputs");
    return;
  }
  auto msg = std_msgs::msg::UInt16MultiArray();
  msg.layout.dim.resize(1);
  msg.layout.dim[0].size = vins.size();
  msg.layout.dim[0].label = "vin";
  msg.layout.dim[0].stride = 1;

  msg.data = vins;
  vin_pub->publish(msg);
}

void Ros2HwPiHat::stop_cb(const std_msgs::msg::Empty::SharedPtr _) {
  board.stopServo(servos);
}

void Ros2HwPiHat::servo_cb(const hw_pi_hat_msgs::msg::Servos::SharedPtr msg) {
  // Parse IDS
  std::vector<uint8_t> ids = msg->ids.data;
  std::vector<uint16_t> positions = msg->positions.data;
  board.setServoPos(ids, positions, 0);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2HwPiHat>());
  rclcpp::shutdown();
  return 0;
}
