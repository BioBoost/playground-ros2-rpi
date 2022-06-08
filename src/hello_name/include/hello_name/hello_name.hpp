#pragma once

#include "rclcpp/rclcpp.hpp"

class HelloNameNode : public rclcpp::Node {
  public:
    // The constructor is a good place to initiate your node with a name and options.
    // You will also define your publishers/subscribers/services here.
    HelloNameNode() : Node("hello_name") {
      RCLCPP_INFO(this->get_logger(), "Hello there from ROS2");

      timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&HelloNameNode::timer_callback, this)
      );
    }

  private:
    void timer_callback() {
      RCLCPP_INFO(this->get_logger(), "Are you still there ? I am ...");
    }

    rclcpp::TimerBase::SharedPtr timer;
};