#ifndef __ROS2_HW_PI_HAT__
#define __ROS2_HW_PI_HAT__

#include "hw_pi_hat_lib/hw_pi_hat_lib.hpp"
#include "hw_pi_hat_msgs/msg/servos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <cstdint>
#include <vector>

class Ros2HwPiHat : public rclcpp::Node {
public:
  Ros2HwPiHat();
  ~Ros2HwPiHat();

private:
  void battery_cb();
  void temp_cb();
  void pos_cb();
  void vin_cb();
  void stop_cb(const std_msgs::msg::Empty::SharedPtr msg);
  void servo_cb(const hw_pi_hat_msgs::msg::Servos::SharedPtr msg);
  std::vector<uint8_t> servos;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr battery_pub;
  rclcpp::TimerBase::SharedPtr battery_timer;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr temp_pub;
  rclcpp::TimerBase::SharedPtr temp_timer;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pos_pub;
  rclcpp::TimerBase::SharedPtr pos_timer;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr vin_pub;
  rclcpp::TimerBase::SharedPtr vin_timer;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscriber;
  rclcpp::Subscription<hw_pi_hat_msgs::msg::Servos>::SharedPtr servo_subscriber;
  hwBoard::Board board;
};

#endif // !__ROS2_HW_PI_HAT__
