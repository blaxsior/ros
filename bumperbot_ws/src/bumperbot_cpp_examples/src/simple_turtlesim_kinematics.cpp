#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"

using std::placeholders::_1;

SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string &name): Node(name) {
  turtle1_pose_sub = create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1));
  turtle2_pose_sub = create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));
}

void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose) {
  last_turtle1_pose = pose;
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose) {
  last_turtle2_pose = pose;

  float Tx = last_turtle2_pose.x - last_turtle1_pose.x;
  float Ty = last_turtle2_pose.y - last_turtle1_pose.y;

  const float pi = 3.141592f;

  float theta_rad = last_turtle2_pose.theta - last_turtle1_pose.theta;
  float theta_deg = 180.0f * theta_rad / pi;

  RCLCPP_INFO_STREAM(get_logger(), "vector Turtle1->Turtle2: (" << Tx << "," << Ty << ")\n" 
  << "[" << std::cos(theta_rad) << ",\t" << -std::sin(theta_rad) << "]\n" 
  << "["<< std::sin(theta_rad) << ",\t" << std::cos(theta_rad) << "]\n" );
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
  rclcpp::spin(node);
  rclcpp::shutdown();
}