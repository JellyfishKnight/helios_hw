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
    std::string motor_name;
    int angle;
    int speed; //rpm
    int current;
    int temperature;
    double states[4];
}MotorState;

typedef struct MotorCommand {

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