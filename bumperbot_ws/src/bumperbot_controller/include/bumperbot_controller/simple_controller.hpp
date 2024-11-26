#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>

class SimpleController: public rclcpp::Node {
public:
  SimpleController(const std::string &name);

private:
  void velCallback(const geometry_msgs::msg::TwistStamped &msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub;

  /** 
   * 바퀴 반지름 길이 
   */
  double wheel_radius;
  /** 
   * 바퀴 사이의 거리 
   */
  double wheel_separation;

  Eigen::Matrix2d speed_conversion;
};

#endif