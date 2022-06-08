#pragma once

#include "rclcpp/rclcpp.hpp"
#include "trex_motor_controller.hpp"

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
      }
      else RCLCPP_ERROR(this->get_logger(), "Failed to initialized trex motor controller");
    }

  private:
    void read_battery() {
      TRex::Status status = motorController.status();
      RCLCPP_INFO(this->get_logger(), "Motor battery voltage: " + std::to_string(status.batteryVoltage));
    }

    rclcpp::TimerBase::SharedPtr timer;
    TRex::TRexMotorController motorController;
};