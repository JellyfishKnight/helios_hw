// created by liuhan on 2023/9/10
#pragma once

#include <cstdint>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"
#include "Resolver.hpp"

#define SERIAL_BAUD 4000000
// #define SERIAL_BAUD 921600
#define SERIAL_NAME "/dev/usb_serial"
#define SERIAL_TIMEOUT 1000
#define MAX_MOTOR_NUMBER 10

namespace helios_control {

const std::vector<std::string> STATE_NAMES{"can_id", "motor_type", "motor_id", "position", "velocity", "current", "temperature"};
const std::vector<std::string> COMMAND_NAMES{"can_id", "motor_type", "motor_id", "motor_value"};

class MotorHardware : public hardware_interface::SystemInterface {
    RCLCPP_SHARED_PTR_DEFINITIONS(MotorHardware);

    MOTOR_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    MOTOR_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    MOTOR_PUBLIC 
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    MOTOR_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    MOTOR_PUBLIC 
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    
    MOTOR_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) override;

    MOTOR_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) override;

    MOTOR_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MOTOR_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MOTOR_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MOTOR_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MOTOR_PUBLIC
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
private: 
    std::shared_ptr<serial::Serial> serial_;
    
    std::vector<WritePacket> write_packet_;
    std::vector<ReadPacket> read_packet_;

    std::vector<HWCommand> hw_commands_;
    std::vector<HWState> hw_states_;

    uint8_t write_buffer_[WRITE_BUFFER_SIZE * MAX_MOTOR_NUMBER];
    uint8_t read_buffer_[READ_BUFFER_SIZE];
    
    rclcpp::Logger logger_ = rclcpp::get_logger("MOTOR_HW");
};

} // namespace helios_control