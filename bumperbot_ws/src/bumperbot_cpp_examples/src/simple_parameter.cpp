#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <vector>

using std::placeholders::_1;

class SimpleParameter: public rclcpp::Node {
public:
  SimpleParameter(): Node("simple_parameter") {
    declare_parameter<int>("simple_int_param", 28);
    declare_parameter<std::string>("simple_string_param", "Default value");
    add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
  }

private:
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

  rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    
    // &-> ref, no copy
    for(const auto& param: params) {
      if(param.get_name() == "simple_int_param" 
      && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed, value = " << param.as_int());
        result.successful = true;
      }

      if(param.get_name() == "simple_string_param" 
      && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed, value = " << param.as_string());
        result.successful = true;
      }
    }

    return result;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc,argv);
  auto simple_parameter = std::make_shared<SimpleParameter>();
  rclcpp::spin(simple_parameter);
  rclcpp::shutdown();

  return 0; 
}