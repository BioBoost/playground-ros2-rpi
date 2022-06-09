#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gamepad_controller.hpp"

class GamepadControlNode : public rclcpp::Node {
  public:
    GamepadControlNode() : Node("gamepad_control") {
      RCLCPP_INFO(this->get_logger(), "Creating Gamepad Control Node");

      if (gamepadController.open()) {
        RCLCPP_INFO(this->get_logger(), "Successfully initialized gamepad controller");

        publisher = this->create_publisher<std_msgs::msg::String>("controllers/gamepad/button", 10);
          // 10 = required queue size to limit messages in the event of a backup. 

        timer = this->create_wall_timer(
          std::chrono::milliseconds(50),
          std::bind(&GamepadControlNode::read_gamepad, this)
        );
      }
      else RCLCPP_ERROR(this->get_logger(), "Failed to initialized gamepad controller");
    }

  private:
    void read_gamepad() {
      struct js_event event;
      if (gamepadController.read_event(&event)) {
        switch (event.type) {
          case JS_EVENT_BUTTON: {
            auto message = std_msgs::msg::String();
            message.data = "Button " + std::to_string(event.number) + (event.value ? " pressed" : " released");
            RCLCPP_INFO(this->get_logger(), "'%s'", message.data.c_str());
            publisher->publish(message);
            break;
          }
          default:
            break;    /* Ignore init events. */
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not read controller. Might be unplugged.");
      }
    }

  private:
    rclcpp::TimerBase::SharedPtr timer;
    Controllers::GamepadController gamepadController;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

};