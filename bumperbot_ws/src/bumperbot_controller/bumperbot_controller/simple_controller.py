#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
  def __init__(self):
    super().__init__("simple_controller_py")

    # param
    self.declare_parameter("wheel_radius", 0.033) # 바퀴 반지름
    self.declare_parameter("wheel_separation", 0.17) # 바퀴 사이 거리

    self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
    self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

    self.get_logger().info(f"using wheel radius: {self.wheel_radius}")
    self.get_logger().info(f"using wheel separation: {self.wheel_separation}")

    self.wheel_cmd_pub = self.create_publisher(
      msg_type=Float64MultiArray, 
      topic="simple_velocity_controller/commands",
      qos_profile=10
    )

    self.vel_sub = self.create_subscription(
      msg_type=TwistStamped,
      topic="bumperbot_controller/cmd_vel",
      callback=self.velCallback,
      qos_profile=10
    )

    self.speed_conversion = np.array([
      [self.wheel_radius/2, self.wheel_radius/2],
      [self.wheel_radius/self.wheel_separation, - self.wheel_radius/self.wheel_separation]]
    )

    self.get_logger().info(f"conversion metrix : {self.speed_conversion}")

  def velCallback(self, msg: TwistStamped):
    # 로봇의 선 각 속도
    robot_speed = np.array([[msg.twist.linear.x],[msg.twist.angular.z]])
    wheel_speed = np.linalg.inv(self.speed_conversion) @ robot_speed

    wheel_speed_msg = Float64MultiArray()
    # 선속도, 각속도
    wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
    self.wheel_cmd_pub.publish(wheel_speed_msg)

def main(args=list[str] | None):
  rclpy.init(args)
  node = SimpleController()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()