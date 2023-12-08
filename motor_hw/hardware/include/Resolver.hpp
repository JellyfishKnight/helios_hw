// created by liuhan,dingzhuo on 2023/9/10
#pragma once

/*               PC协议约定2(ROS)               */
/*
电机包完全参考自（C620) https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf
                (GM6020) https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf
数据0   帧头        0xA0
数据1   CAN类型
        can1   0x01
        can2   0x02
数据2(高8位)+数据3(低8位)   标识符
GM6020电机                  0x1FF     对应数据域 控制电机ID 为 1 2 3 4
                                0x2FF     对应数据域 控制电机ID 为 4 5 6 7
C610/C620电调               0x200     对应数据域 控制电机ID 为 1 2 3 4
                                0x1FF     对应数据域 控制电机ID 为 4 5 6 7
云台电机                      0x140     对应数据域 控制电机ID 为 1
数据4                        id1电机 命令字 0x01 速度控制 0x02 位置控制                      
数据5数据6数据7数据8           数据值
数据9                        id2电机  命令字 0x01 速度控制 0x02 位置控制
数据10数据11数据12数据13       数据值
数据14                       id3电机 命令字 0x01 速度控制 0x02 位置控制
数据15数据16数据17数据18       数据值
数据19                       id4电机 命令字 0x01 速度控制 0x02 位置控制
数据20数据21数据22数据23       数据值
数据24   校验位 (0~23累加)
uint8_t check_sum = 0;
for (uint8_t i = 0; i < usart1_rx_len - 2; i++) {
    check_sum += PC_rx_buff[i];
}
数据25    帧尾   0xA1
*/
/*

//电机
//拉取有效数据位
uint8_t TX_PC_BUFFER[13];
TX_PC_BUFFER[0] = 0xA2;
TX_PC_BUFFER[1] = (uint8_t)(RxHeader.StdId >> 8);
TX_PC_BUFFER[2] = (uint8_t)RxHeadeRxHeaderr.StdId;
memcpy(TX_PC_BUFFER + 3, Data, 8);
RX_PC_BUFFER[11] = can_id
uint8_t check_crc;
for (uint8_t i = 0;i < 12;i++) {
    check_crc += TX_PC_BUFFER[i];
}
TX_PC_BUFFER[12] = check_crc;
TX_PC_BUFFER[13] = 0xA3;


*/
#include <cstddef>
#include <cstdint>
#include <limits>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rcutils/logging.h>
#include <string>
#include <vector>
#include <set>


#define WRITE_BUFFER_SIZE 26
#define READ_BUFFER_SIZE 14

namespace helios_control {

typedef struct HWCommand{
    // double can_id;
    // double motor_type;
    // double motor_id;
    // double motor_mode;
    // double motor_value;
    double cmds[5];
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
    double states[7];
}ReadPacket;

typedef struct WritePacket {
    // can_id;
    // motor_type;
    // motor_mode_1;
    // motor_value_1;
    // motor_mode_2;
    // motor_value_2;
    // motor_mode_3;
    // motor_value_3;
    // motor_mode_4;
    // motor_value_4;
    double cmds[10];
}WritePacket;

class Resolver {
public:
    /**
     * @brief resolve read_buffer to read packet
     * @param motor_state 
     * @param read_buffer 
     * @return true 
     * @return false 
     */
    static bool read_package_resolve(std::vector<HWState>& motor_states, uint8_t *read_buffer) {
        int recv_can_id = read_buffer[11];
        int recv_motor_type = read_buffer[1];
        recv_motor_type = (recv_motor_type << 8) | read_buffer[2];
        int recv_motor_id;
        if (recv_motor_type == 0x201 ||
            recv_motor_type == 0x202 ||
            recv_motor_type == 0x203 ||
            recv_motor_type == 0x204) {
            recv_motor_id = recv_motor_type - 0x200;
            recv_motor_type = 0x200;
        } else if (
            recv_motor_type == 0x205 ||
            recv_motor_type == 0x206 ||
            recv_motor_type == 0x207 ||
            recv_motor_type == 0x208) {
            recv_motor_id = recv_motor_type - 0x204;
            recv_motor_type = 0x1ff;
        } else if (
            recv_motor_type == 0x209 ||
            recv_motor_type == 0x20A ||
            recv_motor_type == 0x20B ||
            recv_motor_type == 0x20C) {
            recv_motor_id = recv_motor_type - 0x204;
            recv_motor_type = 0x1ff;
        } else if (
            recv_motor_type == 0x141 || 
            recv_motor_type == 0x142 ||
            recv_motor_type == 0x143 ||
            recv_motor_type == 0x144) {
            recv_motor_id = recv_motor_type - 0x140;
            recv_motor_type = 0x140;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MotorPacketResolver"), "error info!");
        }
        for (auto &motor_state : motor_states) {
            bool can_id_matched = static_cast<uint8_t>(motor_state.states[0]) == recv_can_id;
            bool motor_type_matched = static_cast<int16_t>(motor_state.states[1]) == recv_motor_type;
            bool motor_id_matched = static_cast<uint8_t>(motor_state.states[2]) == recv_motor_id;
            if (!can_id_matched || !motor_type_matched || !motor_id_matched) {
                // RCLCPP_INFO(rclcpp::get_logger("debug"), 
                //     "\n%d      %x       %d \n%d      %x       %d", 
                //         static_cast<uint8_t>(motor_state.states[0]),
                //         static_cast<int16_t>(motor_state.states[1]),
                //         static_cast<uint8_t>(motor_state.states[2]),
                //         recv_can_id, recv_motor_type, recv_motor_id);
                continue;
            }
            // position
            int temp;
            temp = read_buffer[3];
            temp = (temp << 8) | read_buffer[4];
            motor_state.states[3] = static_cast<double>(temp);
            // velocity
            temp = read_buffer[5];
            temp = (temp << 8) | read_buffer[6];
            motor_state.states[4] = static_cast<double>(temp);
            // current
            temp = read_buffer[7];
            temp = (temp << 8) | read_buffer[8];
            motor_state.states[5] = static_cast<double>(temp);
            // temperature
            temp = read_buffer[9];
            temp = (temp << 8) | read_buffer[10];
            motor_state.states[6] = static_cast<double>(temp);
            return true;
        }
    }
    /**
     * @brief resolve write packet to write buffer
     * @param write_packet 
     * @param write_buffer 
     * @param hw_command 
     * @return true 
     * @return false 
     */
    static bool write_package_resolve(const WritePacket& write_packet, uint8_t *write_buffer, uint8_t num) {
        // frame_head
        write_buffer[0 + WRITE_BUFFER_SIZE * num] = 0xA0;
        // can_id
        write_buffer[1 + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(write_packet.cmds[0]);
        // motor_type
        int temp = static_cast<int>(write_packet.cmds[1]);
        write_buffer[2 + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(temp >> 8);
        write_buffer[3 + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(temp & 0xFF);
        // motor modes and values
        // cause one write packet only have 4 motors
        for (int i = 0; i < 4; i++) {
            // RCLCPP_ERROR(rclcpp::get_logger("debug"), "motor mode %f, motor value %f", write_packet.cmds[2], write_packet.cmds[3 + i * 2]);
            // motor mode
            write_buffer[4 + 5 * i + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(write_packet.cmds[2 + i * 2]);
            // motor value
            temp = static_cast<int32_t>(write_packet.cmds[3 + i * 2]);
            write_buffer[5 + 5 * i  + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(temp >> 24);
            write_buffer[6 + 5 * i  + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(temp >> 16);
            write_buffer[7 + 5 * i  + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(temp >> 8);
            write_buffer[8 + 5 * i  + WRITE_BUFFER_SIZE * num] = static_cast<uint8_t>(temp & 0xff);
        }
        // check_sum
        uint8_t check_sum = 0;
        for (int i = 0 + WRITE_BUFFER_SIZE * num; i < 24 + WRITE_BUFFER_SIZE * num; i++) {
            check_sum += write_buffer[i];
        }
        write_buffer[24 + WRITE_BUFFER_SIZE * num] = check_sum;
        // frame_tail
        write_buffer[25 + WRITE_BUFFER_SIZE * num] = 0xA1;
        return true;
    }

    /**
     * @brief resolve read_buffer to hw_states 
     * @param command 
     * @param write_packet 
     * @return true 
     * @return false 
     */
    [[deprecated("use generate_write_packet instead")]] static bool hw_commands_to_write_packet(HWCommand& command, WritePacket& write_packet) {
        write_packet.cmds[0] = command.cmds[0];
        write_packet.cmds[1] = command.cmds[1];
        // write possibally more motor_values to one write packet
        write_packet.cmds[static_cast<int>(command.cmds[2]) - 1 + 2] = command.cmds[3];
        return true;
    }

    /**
     * @brief resolve read_buffer to hw_states
     *        (maybe should be deprecated)
     * @param read_packet 
     * @param hw_state 
     * @return true 
     * @return false 
     */
    static bool read_packet_to_hw_states(const ReadPacket& read_packet, HWState& hw_state) {
        hw_state.states[0] = read_packet.states[0];
        hw_state.states[1] = read_packet.states[1];
        hw_state.states[2] = read_packet.states[2];
        hw_state.states[3] = read_packet.states[3];
        hw_state.states[4] = read_packet.states[4];
        hw_state.states[5] = read_packet.states[5];
        hw_state.states[6] = read_packet.states[6];
        return true;
    }

    /**
     * @brief check the crc check sum
     * @param read_buffer_ 
     * @return true pass
     * @return false fail
     */
    static bool verify_crc_check_sum(uint8_t *read_buffer_) {
        static int correct_cnt = 0;
        static int wrong_cnt = 0;
        uint8_t check_sum = 0;
        for (int i = 0; i < 12; i++) {
            check_sum += read_buffer_[i];
        }
        if (check_sum != read_buffer_[12]) {
            wrong_cnt++;
            // RCLCPP_WARN(rclcpp::get_logger("debug"), "correct_cnt = %d, wrong_cnt = %d", correct_cnt, wrong_cnt);
            return false;
        }
        correct_cnt++;
        // RCLCPP_WARN(rclcpp::get_logger("debug"), "correct_cnt = %d, wrong_cnt = %d", correct_cnt, wrong_cnt);
        return true;
    }
    /**
     * @brief 减少发送包的数量，统一解析command_interfaces(Need imporve)
     * @param write_packets 
     * @param hw_commands 
     * @return true 
     * @return false 
     */
    static bool generate_write_packet(std::vector<WritePacket>& write_packets, const std::vector<HWCommand>& hw_commands) {
        std::vector<double> can_ids;
        std::vector<std::vector<double>> motor_types;
        // clear write_packets
        write_packets.clear();
        for (std::size_t i = 0; i < hw_commands.size(); i++) {
            // check if can_id exists
            int find_flag = false, find_can_id = 0;
            if (hw_commands[i].cmds[0] == 0) {
                continue;
            }
            for (auto can_id : can_ids) {
                if (can_id == hw_commands[i].cmds[0]) {
                    find_flag = true;
                    find_can_id = static_cast<int>(can_id);
                    break;
                }
            }
            // not exists
            if (!find_flag) {
                // add can_id
                can_ids.push_back(hw_commands[i].cmds[0]);
                // resize to can_id value to avoid out of range
                motor_types.resize(motor_types.size() + static_cast<int>(hw_commands[i].cmds[0]), std::numeric_limits<std::vector<double>>::quiet_NaN());
                // check if motor_type exists
                bool find_motor_type = false;
                for (std::size_t j = 0; j < motor_types[static_cast<int>(hw_commands[i].cmds[0]) - 1].size(); j++) {
                    if (motor_types[static_cast<int>(hw_commands[i].cmds[0]) - 1][j] == hw_commands[i].cmds[1]) {
                        find_motor_type = true;
                        break;
                    }
                }
                // if not exists, add motor_type
                if (!find_motor_type)
                    motor_types[static_cast<int>(hw_commands[i].cmds[0]) - 1].push_back(hw_commands[i].cmds[1]);
            // exists
            } else {
                // check if motor_type exists
                bool find_motor_type = false;
                for (std::size_t j = 0; j < motor_types[find_can_id - 1].size(); j++) {
                    if (motor_types[find_can_id - 1][j] == hw_commands[i].cmds[1]) {
                        find_motor_type = true;
                        break;
                    }
                }
                // if not exists, add motor_type
                if (!find_motor_type)
                    motor_types[find_can_id - 1].push_back(hw_commands[i].cmds[1]);
            }
        }
        // caculate write_packet_number
        int write_packet_number = 0;
        for (std::size_t i = 0; i < motor_types.size(); i++) {
            write_packet_number += motor_types[i].size();
        }
        if (write_packet_number == 0) 
            return true;
        // resize write_packets
        write_packets.resize(write_packet_number);
        // generate write_packets(need improve)
        // write can_id and motor_type
        int i = 0;
        for (auto & can_id : can_ids) {
            for (auto & motor_type : motor_types[static_cast<int>(can_id) - 1]) {
                write_packets[i].cmds[0] = can_id;
                write_packets[i].cmds[1] = motor_type;
                // write possibally more motor_values to one write packet
                for (std::size_t j = 0; j < hw_commands.size(); j++) {
                    if (hw_commands[j].cmds[0] == can_id && hw_commands[j].cmds[1] == motor_type) {
                        write_packets[i].cmds[(static_cast<int>(hw_commands[j].cmds[2]) - 1) * 2 + 2] = hw_commands[j].cmds[3];
                        write_packets[i].cmds[(static_cast<int>(hw_commands[j].cmds[2]) - 1) * 2 + 3] = hw_commands[j].cmds[4];
                    }
                }
                i++;
            }
        }
        return true;
    }
};

} // namespace helios_control