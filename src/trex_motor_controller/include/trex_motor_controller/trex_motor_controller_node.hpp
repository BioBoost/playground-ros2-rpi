#pragma once

#include "rclcpp/rclcpp.hpp"
#include "trex_motor_controller.hpp"
#include "std_msgs/msg/string.hpp"
#include "trex_interfaces/msg/speed.hpp"      // generated ?

class TRexMotorControllerNode : public rclcpp::Node {
  public:
    TRexMotorControllerNode() : Node("trex_motor_controller") {
      RCLCPP_INFO(this->get_logger(), "Starting up TRex Motor Controller Node ...");

      if (motorController.open()) {
        RCLCPP_INFO(this->get_logger(), "Successfully initialized trex motor controller");
        timer = this->create_wall_timer(
          std::chrono::milliseconds(1000),
          std::bind(&TRexMotorControllerNode::read_battery, this)
        );

        moveSubscription = this->create_subscription<trex_interfaces::msg::Speed>(
          "trex/move",          // topic
          10,                   // The depth of the subscription's incoming message queue.
          std::bind(&TRexMotorControllerNode::move_callback, this, std::placeholders::_1)   // callback takes 1 arg
        );
      }
      else RCLCPP_ERROR(this->get_logger(), "Failed to initialized trex motor controller");
    }

  private:
    void read_battery() {
      TRex::Status status = motorController.status();
      RCLCPP_INFO(this->get_logger(), "Motor battery voltage: " + std::to_string(status.batteryVoltage));
    }

    void move_callback(trex_interfaces::msg::Speed::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Move Received: '%d' '%d'", msg->left, msg->right);
      motorController.move(msg->left, msg->right);
    }

    rclcpp::TimerBase::SharedPtr timer;
    TRex::TRexMotorController motorController;
    rclcpp::Subscription<trex_interfaces::msg::Speed>::SharedPtr moveSubscription;
};