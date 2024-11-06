import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class SimpleParameter(Node):
  def __init__(self):
    # init with node name
    super().__init__("simple_parameter")
    # param name / default value
    self.declare_parameter("simple_int_param", 20)
    self.declare_parameter("simple_string_param", "Default Value")

    self.add_on_set_parameters_callback(self.paramChangeCallback)

  def paramChangeCallback(self, params: list[Parameter]) -> SetParametersResult:
    result = SetParametersResult()
    
    for param in params:
      if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
        self.get_logger().info(f"param simple_int_param changed to {param.value}")
        result.successful = True

      elif param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
        self.get_logger().info(f"param simple_int_param changed to {param.value}")
        result.successful = True

    return result
  
def main():
  rclpy.init()
  simple_parameter = SimpleParameter()
  rclpy.spin(simple_parameter)
  simple_parameter.destroy_node()
  rclpy.shutdown()
  
if __name__ == "__main__":
  main()