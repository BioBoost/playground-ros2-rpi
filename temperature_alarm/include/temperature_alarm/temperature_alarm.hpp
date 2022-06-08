#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class TemperatureAlarmNode : public rclcpp::Node {
  public:
    // The constructor is a good place to initiate your node with a name and options.
    // You will also define your publishers/subscribers/services here.
    TemperatureAlarmNode() : Node("temperature_alarm") {
      RCLCPP_INFO(this->get_logger(), "Creating Temperature Alarm Node");

      subscription = this->create_subscription<std_msgs::msg::String>(
        "rpi/temperature",    // topic
        10,                   // The depth of the subscription's incoming message queue.
        std::bind(&TemperatureAlarmNode::topic_callback, this, _1)   // callback takes 1 arg
      );
    }

  private:
    void topic_callback(std_msgs::msg::String::UniquePtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Temperature Received: '%s'", msg->data.c_str());

      double temperature = std::stod(msg->data.c_str());
      if (temperature < 50) RCLCPP_INFO(this->get_logger(), "Temperature of core is good");
      else RCLCPP_WARN(this->get_logger(), "Ohoho - core temperature rising");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

};
