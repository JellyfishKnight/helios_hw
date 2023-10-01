#ifndef REFEREEE_BRIDGE
#define REFEREEE_BRIDGE

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

#include "Referee.hpp"
#include "CRC.hpp"

#include <vector>
#include <map>
#include <string>

#define SOF 0xA5
#define TOF 0xA6

#define SERIAL_PORT_NAME "/dev/ttyUSB1"
#define SERIAL_PORT_BAUDRATE 115200
#define SERIAL_PORT_TIMEOUT 1000


namespace helios_control {

class RefereeBridge : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RefereeBridge);

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC 
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC 
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    REFEREE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    REFEREE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    REFEREE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    REFEREE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
private: 
    std::unique_ptr<serial::Serial> serial_port_;
    std::string serial_port_name_;
    int serial_port_baudrate_;
    int serial_port_timeout_;

    std::unique_ptr<FrameBuffer> header_receive_buffer_;
    
    std::vector<double> game_robot_hp_;
    std::vector<double> power_heat_;
    std::vector<double> shoot_;

    ext_game_robot_HP_t game_robot_hp_data_;
    ext_power_heat_data_t power_heat_data_;
    ext_shoot_data_t shoot_data_;

    std::map<uint16_t, std::function<void(uint8_t *, uint16_t)>> cmd_callback_map_;
    void GameRobotHPCallback(uint8_t * data, uint16_t data_length);
    void PowerHeatDataCallback(uint8_t * data, uint16_t data_length);
    void ShootDataCallback(uint8_t * data, uint16_t data_length);

    rclcpp::Logger logger_ = rclcpp::get_logger("RefereeBridge");
};

} // namespace helios_control

#endif