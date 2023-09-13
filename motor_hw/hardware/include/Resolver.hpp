// created by liuhan on 2023/9/10
#pragma once

/*               PC协议约定3(ROS)               */
/*
数据包完全参考自（C620) https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf
              (GM6020) https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf
数据0   帧头        0xA0
数据1   CAN类型
        can1   0x01
        can2   0x02
//数据2(高8位)+数据3(低8位)   标识符
//GM6020电机                  0x1FF     对应数据域 控制电机ID 为 1 2 3 4
                              0x2FF     对应数据域 控制电机ID 为 4 5 6 7
//C610/C620电调               0x200     对应数据域 控制电机ID 为 1 2 3 4
                              0x1FF     对应数据域 控制电机ID 为 4 5 6 7
//
数据4数据5数据6数据7
数据8数据9数据10数据11    电压或电流值 直接通过CAN转发给电机 具体参考上述PDF

数据12   校验位 (0~11累加)
数据13    帧尾   0xA1

*/


/*

//电机
//拉取有效数据位
uint8_t TX_PC_BUFFER[13];
TX_PC_BUFFER[0] = 0xA2;
TX_PC_BUFFER[1] = (uint8_t)(RxHeader.StdId >> 8);
TX_PC_BUFFER[2] = (uint8_t)RxHeader.StdId;
memcpy(TX_PC_BUFFER + 3, Data, 8);
uint8_t check_crc;
for (uint8_t i = 0;i < 12;i++) {
    check_crc += TX_PC_BUFFER[i];
}
RX_PC_BUFFER[11] = can_id
TX_PC_BUFFER[12] = check_crc;
TX_PC_BUFFER[13] = 0xA3;


*/
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>
#include <string>
#include <vector>

namespace helios_control {

typedef struct HWCommand{
    // double can_id;
    // double motor_type;
    // double motor_id;
    // double motor_value;
    double cmds[4];
}HWCommand;

typedef struct HWState{
    // double can_id;
    // double motor_type;
    // double motor_id;
    // double position;
    // double velocity;
    // double current;
    // double temperature;
    double states[7];
}HWState;

typedef struct ReadPacket {
    double states[6];
}ReadPacket;

typedef struct WritePacket {
    double cmds[6];
}WritePacket;

class Resolver {
public:
    static bool read_package_resolve(std::vector<ReadPacket>& motor_state, uint8_t *read_buffer) {
        
        return true;
    }
    
    static bool write_package_resolve(WritePacket write_packet, uint8_t *write_buffer, HWCommand hw_command) {
        write_buffer[0] = 0xA0;
        write_buffer[1] = static_cast<uint8_t>(write_packet.cmds[0]);
        int temp = static_cast<int>(write_packet.cmds[1]);
        write_buffer[2] = static_cast<uint8_t>(temp >> 8);
        write_buffer[3] = static_cast<uint8_t>(temp & 0xFF);
        // for (int i = 0; i < 4; i++) {
        //     temp = static_cast<int>(write_packet.cmds[i + 2]);
        //     write_buffer[4 + i * 2] = static_cast<uint8_t>(temp >> 8);
        //     write_buffer[5 + i * 2] = static_cast<uint8_t>(temp & 0xFF);
        // }
        temp = static_cast<int>(write_packet.cmds[static_cast<int>(hw_command.cmds[2]) + 1]);
        write_buffer[2 + 2 * static_cast<int>(hw_command.cmds[2])] = static_cast<uint8_t>(temp >> 8);
        write_buffer[3 + 2 * static_cast<int>(hw_command.cmds[2])] = static_cast<uint8_t>(temp & 0xFF);
        uint8_t check_sum = 0;
        for (int i = 0; i < 12; i++) {
            check_sum += write_buffer[i];
        }
        write_buffer[12] = check_sum;
        write_buffer[13] = 0xA1;
        return true;
    }

    static bool hw_commands_to_write_packet(HWCommand& command, WritePacket& write_packet) {
        write_packet.cmds[0] = command.cmds[0];
        write_packet.cmds[1] = command.cmds[1];
        write_packet.cmds[static_cast<int>(command.cmds[2]) - 1 + 2] = command.cmds[3];
        return true;
    }

    static bool read_packet_to_hw_states(ReadPacket& read_packet, HWState& hw_state) {
        hw_state.states[0] = read_packet.states[0];
        hw_state.states[1] = read_packet.states[1];
        hw_state.states[2] = read_packet.states[2];
        hw_state.states[3] = read_packet.states[3];
        hw_state.states[4] = read_packet.states[4];
        hw_state.states[5] = read_packet.states[5];
        return true;
    }

    static bool verify_crc_check_sum(uint8_t *read_buffer_) {
        uint8_t check_sum = 0;
        for (int i = 0; i < 11; i++) {
            check_sum += read_buffer_[i];
        }
        if (check_sum != read_buffer_[11])
            return false;
        return true;
    }
};

} // namespace helios_control