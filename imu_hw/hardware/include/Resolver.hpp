// created by liuhan on 2023/10/7
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace helios_control {

enum class IMUState {
    UART_RVC,
    UART_SHTP,
};

typedef struct IMUPacket {
    IMUState state{IMUState::UART_RVC};
    double x{};
    double y{};
    double z{};
    double w{};
    double x_linear_accel{};
    double y_linear_accel{};
    double z_linear_accel{};
    double x_angular_accel{};
    double y_angular_accel{};
    double z_angular_accel{};
}IMUPacket;

class Resolver {
public:
    /**
     * @brief verify check sum
     * 
     * @param data 
     * @return true 
     * @return false 
     */
    static bool verify_check_sum(const uint8_t * data) {
        #pragma omp simd
        uint8_t sum = 0;
        for (int i = 2; i < 18; i++) {
            sum += data[i];
        }
        return sum == data[18];
    }

    static void read_packet_to_imu_packet(const uint8_t* data, std::vector<IMUPacket>& imu_packets) {
        #pragma omp simd
        if (imu_packets[0].state == IMUState::UART_RVC) {
            
        } else if (imu_packets[0].state == IMUState::UART_SHTP) {
            
        }
    }   

    // 欧拉角转化为四元数
    static void euler_to_quaternion(const double yaw, const double pitch, const double roll, double& x, double& y, double& z, double& w) {
        #pragma omp simd
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        w = cy * cp * cr + sy * sp * sr;
        x = cy * cp * sr - sy * sp * cr;
        y = sy * cp * sr + cy * sp * cr;
        z = sy * cp * cr - cy * sp * sr;
    }
};

} // namespace helios_control