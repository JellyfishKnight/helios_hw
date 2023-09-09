// created by liuhan on 2023/9/10
#include "MotorHardware.hpp"
#include <hardware_interface/actuator_interface.hpp>

namespace helios_control {
hardware_interface::CallbackReturn MotorHardware::on_init(const hardware_interface::HardwareInfo & info) {
    // init info
    if (hardware_interface::ActuatorInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    // resize states
    hw_states_.resize(info_.joints.size());
    for (int i = 0; i < hw_states_.size(); i++) {
        hw_states_[i].motor_name = info_.joints[i].name;
    }
    ///TODO: resize commands
    
    // create serial
    serial_ = std::make_shared<serial::Serial>();
    if (!serial_) {
        RCLCPP_ERROR(logger_, "Unable to create a serial");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
    try {
        // serial_->setPort(info_.hardware_parameters["serial_name"]);
        serial_->setPort("/dev/usb_serial");
        serial_->setBaudrate(115200);
        serial_->setFlowcontrol(serial::flowcontrol_none);
        serial_->setParity(serial::parity_none); // default is parity_none
        serial_->setStopbits(serial::stopbits_one);
        serial_->setBytesize(serial::eightbits);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_->setTimeout(timeout);
    } catch (serial::SerialException& e) {
        RCLCPP_ERROR(logger_, "throwed an exception while declare a serial : %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
    try {
        serial_->open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(logger_, "Got exception while open port: \n %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!serial_->isOpen()) {
        RCLCPP_ERROR(logger_, "Unable to open serial");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // initialize command vector and state vector
    ///TODO: Initialize command vector
    // hw_command_.actuator_current_1 = 0;
    // hw_command_.actuator_current_2 = 0;
    // hw_command_.actuator_current_3 = 0;
    // hw_command_.actuator_current_4 = 0;
    for (auto &state : hw_states_) {
        state.angle = 0;
        state.speed = 0;
        state.current = 0;
        state.temperature = 0;
    }
    RCLCPP_INFO(logger_, "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    serial_->close();
    if (serial_->isOpen()) {
        RCLCPP_ERROR(logger_, "Unable to close serial");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
    serial_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotorHardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                            const std::vector<std::string>& stop_interfaces) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorHardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                            const std::vector<std::string>& stop_interfaces) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    
}

hardware_interface::return_type MotorHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int i = 0; i < hw_states_.size(); i++) {
        hw_states_[i].states[0] = hw_states_[i].angle;
        hw_states_[i].states[1] = hw_states_[i].speed;
        hw_states_[i].states[2] = hw_states_[i].current;
        hw_states_[i].states[3] = hw_states_[i].temperature;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "angle", &hw_states_[i].states[0]
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "speed", &hw_states_[i].states[1]
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "current", &hw_states_[i].states[2]
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "temperature", &hw_states_[i].states[3]
        ));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    ///TODO: Export command interfaces
    // for (int i = 0; i < 4; i++) {
    //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //         info_.joints[i].name, "current", &hw_command_.cmds[i]
    //     ));
    // }
    return command_interfaces;
}

hardware_interface::CallbackReturn MotorHardware::on_error(const rclcpp_lifecycle::State & previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}


} // helios_control


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  helios_control::MotorHardware,
  hardware_interface::ActuatorInterface)