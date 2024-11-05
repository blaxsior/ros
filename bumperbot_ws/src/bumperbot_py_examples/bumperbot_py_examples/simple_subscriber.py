import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
  def __init__(self):
    super().__init__("simple_subscriber")
    self.sub = self.create_subscription(String, "/chatter", self.msgCallback, 10)

  def msgCallback(self, msg: String):
    self.get_logger().info(f"message: {msg.data}")
  
def main():
  rclpy.init()
  simple_sub = SimpleSubscriber()
  rclpy.spin(simple_sub)
  simple_sub.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()