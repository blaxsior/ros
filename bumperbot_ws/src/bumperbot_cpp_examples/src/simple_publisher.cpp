#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher: public rclcpp::Node
{
public:
  SimplePublisher(): Node("simple_publisher"), counter(0) {
    pub = create_publisher<std_msgs::msg::String>("/chatter", 10);
    //std::chrono::seconds(1)
    timer = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));

    RCLCPP_INFO(get_logger(), "publishing at 1 Hz");
  }
private:
  unsigned int counter;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;

  void timerCallback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS 2 - counter: " + std::to_string(counter++);

    pub->publish(message);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePublisher>();
  rclcpp::spin(node);
  // shared_ptr이라 따로 free 안해도 됨
  rclcpp::shutdown();

  return 0;
}