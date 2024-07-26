#pragma once

enum ErrorType {
    kPointerError,
    kHalLibError,
    kSwitchError,
    kMotorError,
    kDr16Error,
    kImuError,
    kComuError,
    kVisionError,
};

void ErrorHandle(enum ErrorType error_type);
