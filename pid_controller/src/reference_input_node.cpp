#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "pid_controller_msgs/srv/set_reference.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("reference_input_node");
  auto client = node->create_client<pid_controller_msgs::srv::SetReference>("set_reference");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  std::string input;

  while (rclcpp::ok()) {
    std::cout << "Skriv ny referenceverdi (eller q for å avslutte): ";
    std::getline(std::cin, input);

    if (!rclcpp::ok()) {
      break;
    }

    if (input == "q" || input == "quit" || input == "exit") {
      break;
    }

    try {
      double value = std::stod(input);

      auto request = std::make_shared<pid_controller_msgs::srv::SetReference::Request>();

      request->request = value;

      auto future = client->async_send_request(request);

      if (rclcpp::spin_until_future_complete(node, future) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        auto response = future.get();

        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "Reference satt til %.3f", value);
        } else {
          RCLCPP_WARN(node->get_logger(), "Server avviste reference %.3f", value);
        }
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service set_reference");
      }
    }
    catch (const std::exception &) {
      RCLCPP_WARN(node->get_logger(), "Ugyldig input. Skriv et tall.");
    }
  }

  rclcpp::shutdown();
  return 0;
}