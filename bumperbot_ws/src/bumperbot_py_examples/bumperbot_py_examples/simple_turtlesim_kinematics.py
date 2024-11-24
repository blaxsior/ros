import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import pi, cos, sin

class SimpleTurtleSimKinematics(Node):
  def __init__(self):
    super().__init__("simple_turtlesim_kinematics")
    self.t1_pose_sub = self.create_subscription(
      msg_type=Pose, 
      topic="/turtle1/pose", 
      callback=self.turtle1PoseCallback,
      qos_profile=10)
    
    self.t2_pose_sub = self.create_subscription(
      msg_type=Pose, 
      topic="/turtle2/pose", 
      callback=self.turtle2PoseCallback,
      qos_profile=10)
    
    self.last_turtle1_pose = Pose()
    self.last_turtle2_pose = Pose()

  def turtle1PoseCallback(self, msg: Pose):
    self.last_turtle1_pose = msg

  def turtle2PoseCallback(self, msg: Pose):
    self.last_turtle2_pose = msg

    Tx = self.last_turtle2_pose.x - self.last_turtle1_pose.x
    Ty = self.last_turtle2_pose.y - self.last_turtle1_pose.y

    theta_rad = self.last_turtle2_pose.theta - self.last_turtle1_pose.theta
    theta_deg = 180.0 * theta_rad / pi

    self.get_logger().info(f"Vector[t1 -> t2]:\n ({Tx:.3}, {Ty:.3}, {theta_deg}', \n{cos(theta_rad)} {-sin(theta_rad)}\n {sin(theta_rad)} {cos(theta_rad)})\n")


def main():
  rclpy.init()
  node = SimpleTurtleSimKinematics()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()