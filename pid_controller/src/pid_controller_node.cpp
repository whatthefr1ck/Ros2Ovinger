#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pid_controller_msgs/srv/set_reference.hpp"

using namespace std::chrono_literals;

class pidController
{
public:
  double p{2.0};
  double i{0.2};
  double d{0.1};

  double reference{5.0};
  double voltage{0.0};

  double integral{0.0};
  double prev_error{0.0};
  bool first{true};

  void update(double measured_angle, double dt)
  {
    const double error = reference - measured_angle;
    integral += error * dt;

    double derivative = 0.0;
    if (!first && dt > 0.0) {
      derivative = (error - prev_error) / dt;
    } else {
      first = false;
    }

    voltage = p * error + i * integral + d * derivative;
    prev_error = error;
  }
};

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode()
  : Node("pid_controller_node")
  {
    this->declare_parameter("p", 2.0);
    this->declare_parameter("i", 0.2);
    this->declare_parameter("d", 0.1);

    pid_.p = this->get_parameter("p").as_double();
    pid_.i = this->get_parameter("i").as_double();
    pid_.d = this->get_parameter("d").as_double();

    cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PIDControllerNode::parameter_callback, this, std::placeholders::_1)
    );

    publish_voltage_ =
      this->create_publisher<std_msgs::msg::Float64>("voltage", 10);

    measured_angle_ =
      this->create_subscription<std_msgs::msg::Float64>(
        "measured_angle", 10,
        std::bind(&PIDControllerNode::measurement_listener, this, std::placeholders::_1)
      );

    set_reference_service_ =
      this->create_service<pid_controller_msgs::srv::SetReference>(
        "set_reference",
        std::bind(
          &PIDControllerNode::set_reference_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    dt_ = 0.01;
    timer_ = this->create_wall_timer(10ms, std::bind(&PIDControllerNode::on_timer, this));
  }

private:
  void measurement_listener(const std_msgs::msg::Float64::SharedPtr msg)
  {
    last_measured_angle_ = msg->data;
    have_measurement_ = true;
    pid_.update(last_measured_angle_, dt_);
    publish_voltage();
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : parameters) {
      if (p.get_name() == "p") {
        pid_.p = p.as_double();
      } else if (p.get_name() == "i") {
        pid_.i = p.as_double();
      } else if (p.get_name() == "d") {
        pid_.d = p.as_double();
      }
    }

    return result;
  }

  void on_timer()
  {
    pid_.update(last_measured_angle_, dt_);
    publish_voltage();
  }

  void publish_voltage()
  {
    std_msgs::msg::Float64 out;
    out.data = pid_.voltage;
    publish_voltage_->publish(out);
  }

  void set_reference_callback(
    const std::shared_ptr<pid_controller_msgs::srv::SetReference::Request> request,
    std::shared_ptr<pid_controller_msgs::srv::SetReference::Response> response)
  {
    bool success = request->request > -3.14 && request->request < 3.14;

    if (success) {
      pid_.reference = request->request;
      RCLCPP_INFO(this->get_logger(), "New reference: %.3f", pid_.reference);
    }

    response->success = success;
  }

  pidController pid_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publish_voltage_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_angle_;
  rclcpp::Service<pid_controller_msgs::srv::SetReference>::SharedPtr set_reference_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
  rclcpp::TimerBase::SharedPtr timer_;

  double dt_{0.01};
  double last_measured_angle_{0.0};
  bool have_measurement_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}