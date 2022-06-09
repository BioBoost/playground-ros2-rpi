#include "rclcpp/rclcpp.hpp"
#include "gamepad_control/gamepad_control_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadControlNode>());
  rclcpp::shutdown();
  return 0;
}