#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trex_interfaces/msg/speed.hpp"

class ThumperDriveNode : public rclcpp::Node {
  public:
    ThumperDriveNode() : Node("thumper_drive") {
      RCLCPP_INFO(this->get_logger(), "Starting Thumper drive Node ...");

      gamepadSubscriber = this->create_subscription<std_msgs::msg::String>(
        "controllers/gamepad/button",          // topic
        10,                   // The depth of the subscription's incoming message queue.
        std::bind(&ThumperDriveNode::gamepad_callback, this, std::placeholders::_1)   // callback takes 1 arg
      );

      trexPublisher = this->create_publisher<trex_interfaces::msg::Speed>("trex/move", 10);
    }

  private:
    void gamepad_callback(std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Gamepad Received: '%s'", msg->data.c_str());

      auto message = trex_interfaces::msg::Speed();
      if (msg->data.compare("Button 1 pressed") == 0) {
        message.left = 50;
        message.right = 50;
        trexPublisher->publish(message);
      } else if (msg->data.compare("Button 1 released") == 0) {
        message.left = 0;
        message.right = 0;
        trexPublisher->publish(message);
      }
    }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamepadSubscriber;
    rclcpp::Publisher<trex_interfaces::msg::Speed>::SharedPtr trexPublisher;
};