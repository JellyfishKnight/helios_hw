#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

#include "GM6020.hpp"

namespace helios_control {
class GM6020Hardware : public hardware_interface::ActuatorInterface {
    RCLCPP_SHARED_PTR_DEFINITIONS(GM6020Hardware);

    GM6020_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    GM6020_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    GM6020_PUBLIC 
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    GM6020_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    GM6020_PUBLIC 
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    
    GM6020_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) override;

    GM6020_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) override;

    GM6020_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    GM6020_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    GM6020_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    GM6020_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    GM6020_PUBLIC
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
private: 
    std::shared_ptr<serial::Serial> serial_;

    GM6020_Cmd hw_command_;
    std::vector<GM6020_State> hw_states_;

    uint8_t read_buffer_[256];
    uint8_t write_buffer[10];

    rclcpp::Logger logger_ = rclcpp::get_logger("GM6020");
};




} // namespace helios_control