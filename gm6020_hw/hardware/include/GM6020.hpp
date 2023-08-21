#pragma once

#include <cstdint>
#include <vector>

#define HW_IF_CURRENT "current"
#define HW_IF_ANGLE "angle"
#define HW_IF_SPEED "speed"
#define HW_IF_EFFORT "effort"
#define HW_IF_TEMP "temperature"

typedef struct GM6020Command {
    // head is 0x1ff
    int actuator_current_1;
    int actuator_current_2;
    int actuator_current_3;
    int actuator_current_4;
}GM6020_Cmd;

typedef struct GM6020State {
    // head is 0x204 + number
    // 0-8191
    int angle;
    // rpm
    int speed;
    int effort;
    int temperature;
    // converter funtion
}GM6020_State;

inline void convert_read_buffer_to_states(uint8_t *read_buffer, std::vector<GM6020_State>& states) {
    // get head
    int sign = read_buffer[0];
    sign = sign << 8 | read_buffer[1];
    // actuator 1, 2...
    if (sign == 0x205) {
        states[0].angle = read_buffer[2];
        states[0].angle = (states[0].angle << 8) | read_buffer[3];
        states[0].angle = read_buffer[4];
        states[0].angle = (states[0].angle << 8) | read_buffer[5];
        states[0].angle = read_buffer[2];
        states[0].angle = (states[0].angle << 8) | read_buffer[7];
        states[0].temperature = read_buffer[8];
    } else if (sign == 0x206) {
        states[1].angle = read_buffer[2];
        states[1].angle = (states[0].angle << 8) | read_buffer[3];
        states[1].angle = read_buffer[4];
        states[1].angle = (states[0].angle << 8) | read_buffer[5];
        states[1].angle = read_buffer[2];
        states[1].angle = (states[0].angle << 8) | read_buffer[7];
        states[1].temperature = read_buffer[8];
    } else if (sign == 0x207) {
        states[2].angle = read_buffer[2];
        states[2].angle = (states[0].angle << 8) | read_buffer[3];
        states[2].angle = read_buffer[4];
        states[2].angle = (states[0].angle << 8) | read_buffer[5];
        states[2].angle = read_buffer[2];
        states[2].angle = (states[0].angle << 8) | read_buffer[7];
        states[2].temperature = read_buffer[8];
    } else if (sign == 0x208) {
        states[3].angle = read_buffer[2];
        states[3].angle = (states[0].angle << 8) | read_buffer[3];
        states[3].angle = read_buffer[4];
        states[3].angle = (states[0].angle << 8) | read_buffer[5];
        states[3].angle = read_buffer[2];
        states[3].angle = (states[0].angle << 8) | read_buffer[7];
        states[3].temperature = read_buffer[8];
    }
}

inline void convert_command_to_write_buffer(GM6020_Cmd& cmd, uint8_t* buffer) {
    buffer[0] = 0x01;
    buffer[1] = 0xff;
    buffer[3] = cmd.actuator_current_1;
    buffer[2] = cmd.actuator_current_1 >> 8;
    buffer[5] = cmd.actuator_current_2;
    buffer[4] = cmd.actuator_current_2 >> 8;
    buffer[7] = cmd.actuator_current_3;
    buffer[6] = cmd.actuator_current_3 >> 8;
    buffer[9] = cmd.actuator_current_4;
    buffer[8] = cmd.actuator_current_4 >> 8;
}