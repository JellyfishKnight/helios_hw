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
#include "IMUHardware.hpp"


namespace helios_control {

hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) {
    
}

hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) {
    
}

hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) {
    
}

hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    
}

hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) {
    
}

hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    
}

std::vector<hardware_interface::StateInterface> export_state_interfaces() {
    
}

hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) {
    
}

} // namespace helios_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  helios_control::IMUHardware,
  hardware_interface::SensorInterface)