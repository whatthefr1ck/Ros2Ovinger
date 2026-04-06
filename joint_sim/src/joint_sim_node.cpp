#include <chrono>
#include <memory>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

// jointSimulator (basic)
class jointSimulator
{
public:
  double angle{0.0};
  double angular_velocity{0.0};
  double voltage{0.0};
  double noise{0.0}; // foreløpig 0

  // Konstanter
  double K = 230.0;
  double T = 0.15;

  void update(double dt)
  {
    // T * domega/dt + omega = K*V  => domega/dt = (K*V - omega)/T
    const double domega = (K * voltage - angular_velocity) / T;
    angular_velocity += dt * domega;

    // dtheta/dt = omega
    angle += dt * angular_velocity;

    // noise holdes 0 foreløpig
  }
};

class JointSimulatorNode : public rclcpp::Node
{
public:
  JointSimulatorNode()
  : Node("joint_simulator_node")
  {

    this->declare_parameter("noise", 1.0);
    this->declare_parameter("K", 230.0);
    this->declare_parameter("T", 0.15);
    sim_.noise = this->get_parameter("noise").as_double();
    sim_.K = this->get_parameter("K").as_double();
    sim_.T = this->get_parameter("T").as_double();
    cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&JointSimulatorNode::parameter_callback, this, std::placeholders::_1)
    );
    
    
    publish_angle = this->create_publisher<std_msgs::msg::Float64>("measured_angle", 10);
    input_voltage = this->create_subscription<std_msgs::msg::Float64>(
      "voltage", 10,
      std::bind(&JointSimulatorNode::voltage_listener, this, std::placeholders::_1)
    );
    // Timer som oppdaterer simulator og publiserer vinkel
    dt_ = 0.01; // 10ms
    timer_ = this->create_wall_timer(10ms, std::bind(&JointSimulatorNode::on_timer, this));
  }

private:
  void voltage_listener(const std_msgs::msg::Float64::SharedPtr msg)
  {
    sim_.voltage = msg->data;
  }
  
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : parameters) {
      if (p.get_name() == "noise") sim_.noise = p.as_double();
      else if (p.get_name() == "K") sim_.K = p.as_double();
      else if (p.get_name() == "T") sim_.T = p.as_double();
    }
    return result;
  }

  
  void on_timer()
  {
    sim_.update(dt_);

    std_msgs::msg::Float64 out;
    out.data = sim_.angle;
    publish_angle->publish(out);
  }

  jointSimulator sim_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publish_angle;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr input_voltage;

  double dt_{0.01};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}