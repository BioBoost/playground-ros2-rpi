#pragma once

#include <unistd.h>

namespace TRex {

  struct Command {

    int16_t leftMotorSpeed = 0;
    uint8_t leftMotorBrake = 0;
    int16_t rightMotorSpeed = 0;
    uint8_t rightMotorBrake = 0;
    double batteryThreshold = 5.8;

  };

}