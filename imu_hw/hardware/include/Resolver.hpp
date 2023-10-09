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

typedef struct RVCRawData {
    double yaw;
    double pitch;
    double roll;
    double x_axis_accel;
    double y_axis_accel;
    double z_axis_accel;
}RVCRawData;

typedef struct SHTPRawData {

}SHTPRawData;

typedef struct IMUPacket {
    IMUState state{IMUState::UART_RVC};
    double x{};
    double y{};
    double z{};
    double w{};
    double x_linear_accel{};
    double y_linear_accel{};
    double z_linear_accel{};
    double x_angular_vel{};
    double y_angular_vel{};
    double z_angular_vel{};
    double yaw{};
    double pitch{};
    double roll{};
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

    static void read_packet_to_imu_packet(const uint8_t* data, RVCRawData& imu_packets) {
        #pragma omp simd
        imu_packets.yaw = static_cast<int16_t>(data[3] | (data[4] << 8)) / 100.0;
        imu_packets.pitch = static_cast<int16_t>(data[5] | (data[6] << 8)) / 100.0;
        imu_packets.roll = static_cast<int16_t>(data[7] | (data[8] << 8)) / 100.0;
        imu_packets.x_axis_accel = static_cast<int16_t>(data[9] | (data[10] << 8)) * 0.0097913;
        imu_packets.y_axis_accel = static_cast<int16_t>(data[11] | (data[12] << 8)) * 0.0097913;
        imu_packets.z_axis_accel = static_cast<int16_t>(data[13] | (data[14] << 8)) * 0.0097913;
    }   

    static void read_packet_to_imu_packet(const uint8_t* data, SHTPRawData& imu_packets) {
        #pragma omp simd
        
    }   

    // 欧拉角转化为四元数
    static void euler_to_quaternion(double yaw, double pitch, double roll, double& x, double& y, double& z, double& w) {
        #pragma omp simd
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        w = cy * cp * cr + sy * sp * sr;
        x = cy * cp * sr - sy * sp * cr;
        y = sy * cp * sr + cy * sp * cr;
        z = sy * cp * cr - cy * sp * sr;
    }

    static void raw_to_imu_packet(IMUPacket& packet, const RVCRawData& raw_data, const rclcpp::Time & time) {
        #pragma omp simd
        euler_to_quaternion(raw_data.yaw, raw_data.pitch, raw_data.roll, packet.x, packet.y, packet.z, packet.w);
        // caculate acceleration
        packet.x_linear_accel = raw_data.x_axis_accel;
        packet.y_linear_accel = raw_data.y_axis_accel;
        packet.z_linear_accel = raw_data.z_axis_accel;
        // caculate angular velocity
        // get length of time slice
        double gap_time = 0.001;
        last_time_ = time;  
        double yaw_diff, pitch_diff, roll_diff;
        yaw_diff = raw_data.yaw - last_rvc_raw_.yaw;
        pitch_diff = raw_data.pitch - last_rvc_raw_.pitch;
        roll_diff = raw_data.roll - last_rvc_raw_.roll;
        packet.x_angular_vel = roll_diff / gap_time;
        packet.y_angular_vel = pitch_diff / gap_time;
        packet.z_angular_vel = yaw_diff / gap_time;
        packet.yaw = raw_data.yaw;
        packet.pitch = raw_data.pitch;
        packet.roll = raw_data.roll;
    }

    static void raw_to_imu_packet(IMUPacket& packet, const SHTPRawData& raw_data, const rclcpp::Time & time) {

    }

public:
    static rclcpp::Time last_time_;
    static RVCRawData last_rvc_raw_;
    static SHTPRawData last_shtp_raw_;
};


} // namespace helios_control