#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>

class TemperaturePublisherNode : public rclcpp::Node {
  public:
    // The constructor is a good place to initiate your node with a name and options.
    // You will also define your publishers/subscribers/services here.
    TemperaturePublisherNode() : Node("temperature_publisher") {
      RCLCPP_INFO(this->get_logger(), "Creating Temperature Publisher Node");

      publisher = this->create_publisher<std_msgs::msg::String>("rpi/temperature", 10);
        // 10 = required queue size to limit messages in the event of a backup. 

      timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&TemperaturePublisherNode::publish_temperature, this)
      );
    }

  private:
    void publish_temperature() {
      auto message = std_msgs::msg::String();
      message.data = std::to_string(this->get_temperature());
      RCLCPP_INFO(this->get_logger(), "Publishing RPi temperature: '%s'", message.data.c_str());
      publisher->publish(message);
    }

    double get_temperature() {
      std::ifstream file;
      file.open("/sys/class/thermal/thermal_zone0/temp");

      double temperature = 0;
      if (file.is_open()) {
        std::string text;
        file >> text;
        temperature = std::stod(text, nullptr)/1000;
      }
      
      return temperature;
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};