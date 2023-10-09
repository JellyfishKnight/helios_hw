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

#include <hardware_interface/sensor_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <serial/serial.h>
#include <vector>

#include "visibility_control.h"
#include "Resolver.hpp"

#define SERIAL_BAUD 115200
#define SERIAL_NAME "/dev/ch340_serial"
#define SERIAL_TIMEOUT 1000

#define MAX_READ_LENGTH 256

#define X "x"
#define Y "y"
#define Z "z"
#define W "w"
#define X_LINEAR_ACCEL "x_linear_accel"
#define Y_LINEAR_ACCEL "y_linear_accel"
#define Z_LINEAR_ACCEL "z_linear_accel"
#define X_ANGULAR_VEL "x_angular_vel"
#define Y_ANGULAR_VEL "y_angular_vel"
#define Z_ANGULAR_VEL "z_angular_vel"
#define YAW "yaw"
#define PITCH "pitch"
#define ROLL "roll"


namespace helios_control {

#define IMU_MODE IMUState::UART_RVC


class IMUHardware : public hardware_interface::SensorInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IMUHardware);

    IMU_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    IMU_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    IMU_PUBLIC 
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    IMU_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    IMU_PUBLIC 
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    
    IMU_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    IMU_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    IMU_PUBLIC
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
    std::shared_ptr<serial::Serial> serial_;
    
    std::vector<IMUPacket> imu_packets_;
    std::vector<SHTPRawData> shtp_raw_packets_;
    std::vector<RVCRawData> rvc_raw_packets_;

    uint8_t read_buffer_[MAX_READ_LENGTH];

    rclcpp::Logger logger_ = rclcpp::get_logger("IMUHardware");
};

} // namespace helios_control