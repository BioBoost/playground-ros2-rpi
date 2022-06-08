#pragma once

#include "rclcpp/rclcpp.hpp"

class HelloNameNode : public rclcpp::Node {
  public:
    HelloNameNode() : Node("hello_name") {
      RCLCPP_INFO(this->get_logger(), "Hello there from ROS2");
    }
  private:
};