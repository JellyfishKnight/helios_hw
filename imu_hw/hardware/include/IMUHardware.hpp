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

namespace helios_control {

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


    rclcpp::Logger logger_ = rclcpp::get_logger("IMUHardware");
};

} // namespace helios_control