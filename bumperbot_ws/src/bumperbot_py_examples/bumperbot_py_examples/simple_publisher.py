import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
  def __init__(self):
    # name of this node is simple_publisher
    super().__init__("simple_publisher")
    # msg_type 송수신할 데이터 타입
    # topic 토픽 이름
    # qos_profile 메시지 큐 사이즈
    self.pub = self.create_publisher(msg_type=String, topic="chatter",qos_profile=10)
    self.counter = 0
    self.frequency = 1.0 # 메시지 게시 속도

    self.get_logger().info(f"publishing at {self.frequency} Hz")

    # 타이머 생성
    self.timer = self.create_timer(self.frequency, self.timerCallback)

  def timerCallback(self):
    msg = String()
    msg.data = f"hello, ros2 - counter = {self.counter}"

    self.pub.publish(msg = msg)
    self.counter += 1


def main():
  rclpy.init() # rclpy 시작. 노드 초기화, 통신 인프라 / ROS 매개변수 / 로깅 / 글로벌 리소스 / 명령 인자 처리 등
  simple_publisher = SimplePublisher() 
  rclpy.spin(simple_publisher) # 연속적으로 노드에 대한 이벤트 처리 수행
  simple_publisher.destroy_node() # 노드를 제거
  rclpy.shutdown() # ros2와 관련된 모든 리소스 정리

if __name__ == "__main__":
  main()