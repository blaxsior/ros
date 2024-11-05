#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleSubscriber: public rclcpp::Node {
public:
  SimpleSubscriber(): Node("simple_subscriber") {
    sub = create_subscription<std_msgs::msg::String>("/chatter", 10, std::bind(
      &SimpleSubscriber::messageCallback, 
      this, 
      std::placeholders::_1)); // one parameter 
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

  void messageCallback(const std_msgs::msg::String msg) const {
    RCLCPP_INFO_STREAM(get_logger(), "hello " << msg.data.c_str()); // 두 문자열을 cstr로 붙여 출력
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSubscriber>();
  rclcpp::spin(node); // wait for events
  rclcpp::shutdown(); // shutdown
  return 0;
}