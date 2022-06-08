// This node does just one thing: say Hello when it’s started, and that’s it. Then it spins until you kill it.

#include "rclcpp/rclcpp.hpp"
#include "hello_name/hello_name.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloNameNode>();

  rclcpp::spin(node);
    // Pause the program here and keep the node alive. When you request to kill the node (for example CTRL+C), the spinning will stop and the program will resume.
    // Also it will allow all defined callback functions to be called. If you create a callback for a subscriber, a parameter, etc., then spin will monitor any input coming from other nodes, and will trigger some callbacks if needed.

  // After you’ve killed the node, this is what gets executed. rclcpp::shutdown() will stop ROS2 communications, it is basically the opposite of rclcpp::init().
  rclcpp::shutdown();

  return 0;
}