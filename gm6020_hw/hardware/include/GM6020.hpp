#pragma once

#include <cstdint>
#include <iostream>
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
    double cmds[4];
}GM6020_Cmd;

typedef struct GM6020State {
    // head is 0x204 + number
    // 0-8191
    int angle;
    // rpm
    int speed;
    int effort;
    int temperature;
    double states[4];
}GM6020_State;

inline void convert_read_buffer_to_states(uint8_t *read_buffer, std::vector<GM6020_State>& states) {
    // get head
    int sign = read_buffer[0];
    sign = (sign << 8) | read_buffer[1];
    // actuator 1, 2...
    // 实际应该是0x205
    // 这里测试3508改为0x201
    if (sign == 0x201) {
        states[0].angle = read_buffer[2];
        states[0].angle = (states[0].angle << 8) | read_buffer[3];
        states[0].speed = read_buffer[4];
        states[0].speed = (states[0].speed << 8) | read_buffer[5];
        states[0].effort = read_buffer[6];
        states[0].effort = (states[0].effort << 8) | read_buffer[7];
        states[0].temperature = read_buffer[8];
    } else if (sign == 0x206) {
        states[1].angle = read_buffer[2];
        states[1].angle = (states[1].angle << 8) | read_buffer[3];
        states[1].speed = read_buffer[4];
        states[1].speed = (states[1].speed << 8) | read_buffer[5];
        states[1].effort = read_buffer[6];
        states[1].effort = (states[1].effort << 8) | read_buffer[7];
        states[1].temperature = read_buffer[8];
    } else if (sign == 0x207) {
        states[2].angle = read_buffer[2];
        states[2].angle = (states[2].angle << 8) | read_buffer[3];
        states[2].speed = read_buffer[4];
        states[2].speed = (states[2].speed << 8) | read_buffer[5];
        states[2].effort = read_buffer[6];
        states[2].effort = (states[2].effort << 8) | read_buffer[7];
        states[2].temperature = read_buffer[8];
    } else if (sign == 0x208) {
        states[3].angle = read_buffer[2];
        states[3].angle = (states[3].angle << 8) | read_buffer[3];
        states[3].speed = read_buffer[4];
        states[3].speed = (states[3].speed << 8) | read_buffer[5];
        states[3].effort = read_buffer[6];
        states[3].effort = (states[3].effort << 8) | read_buffer[7];
        states[3].temperature = read_buffer[8];
    }
}

inline void convert_command_to_write_buffer(GM6020_Cmd& cmd, uint8_t* buffer) {
    cmd.actuator_current_1 = cmd.cmds[0];
    cmd.actuator_current_2 = cmd.cmds[1];
    cmd.actuator_current_3 = cmd.cmds[2];
    cmd.actuator_current_4 = cmd.cmds[3];
    // 这个才是6020的表示符
    // buffer[0] = 0x01;
    // buffer[1] = 0xff;
    // 测试3508电机
    buffer[0] = 0x02;
    buffer[1] = 0x00;
    buffer[2] = cmd.actuator_current_1 >> 8 & 0xff;
    buffer[3] = cmd.actuator_current_1 & 0xff;
    buffer[4] = cmd.actuator_current_2 >> 8 & 0xff;
    buffer[5] = cmd.actuator_current_2 & 0xff;
    buffer[6] = cmd.actuator_current_3 >> 8 & 0xff;
    buffer[7] = cmd.actuator_current_3 & 0xff;
    buffer[8] = cmd.actuator_current_4 >> 8 & 0xff;
    buffer[9] = cmd.actuator_current_4 & 0xff;
}