// created by liuhan on 2023/9/10
#pragma once

#include <cstdint>
#include <string>

#define HW_IF_ANGLE "angle"
#define HW_IF_SPEED "speed"
#define HW_IF_CURRENT "current"
#define HW_IF_TEMP "temperature"

namespace helios_control {

typedef struct MotorState {
    // yaw pitch chassis_1 ...
    std::string motor_name;
    int angle;
    int speed; //rpm
    int current;
    int temperature;
    double states[4];
}MotorState;

typedef struct MotorCommand {
    // yaw pitch chassis_1 ...
    std::string motor_name;
    // in speed mode, value is rpm
    // in angle mode, value = value * 1/8196 round
    float value;
    int motor_id;
    bool is_speed_mode;
    float kp;
    float ki;
    float kd;
    float i_limit;
    float filter;
    double commands[8];
}MotorCommand;

class MotorResolver {
public:
    static bool read_package_resolve(uint8_t *read_buffer) {

        return true;
    }

    static bool write_package_resolve(uint8_t *read_buffer) {

        return true;
    }
};

} // namespace helios_control