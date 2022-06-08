#pragma once

#include <unistd.h>
#include "trex_error_flags.hpp"
#include "trex_operation_mode.hpp"

namespace TRex {

  struct Status {

    ErrorFlags errors = ErrorFlags::NONE;
    double batteryVoltage = 0;
    double leftMotorCurrent = 0;
    double rightMotorCurrent = 0;
    OperationMode mode = OperationMode::ALL_GOOD;

  };

}