// created by liuhan on 2023/9/10
#include "MotorHardware.hpp"
#include "Resolver.hpp"
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <set>
#include <string>

namespace helios_control {
hardware_interface::CallbackReturn MotorHardware::on_init(const hardware_interface::HardwareInfo & info) {
    // init info
    if (hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // resize states and commands
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<HWCommand>::quiet_NaN());
    hw_states_.resize(info_.joints.size(), std::numeric_limits<HWState>::quiet_NaN());
    write_packet_.resize(info_.joints.size(), std::numeric_limits<WritePacket>::quiet_NaN());
    read_packet_.resize(info_.joints.size(), std::numeric_limits<ReadPacket>::quiet_NaN());
    memset(write_buffer_, 0, sizeof(write_buffer_));
    memset(read_buffer_, 0, sizeof(read_buffer_));
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
        serial_->setPort(SERIAL_NAME);
        serial_->setBaudrate(SERIAL_BAUD);
        serial_->setFlowcontrol(serial::flowcontrol_none);
        serial_->setParity(serial::parity_none); // default is parity_none
        serial_->setStopbits(serial::stopbits_one);
        serial_->setBytesize(serial::eightbits);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT);
        serial_->setTimeout(timeout);
    } catch (serial::SerialException& e) {
        RCLCPP_ERROR(logger_, "throwed an exception while declare a serial : %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
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


hardware_interface::return_type MotorHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // read motor state packet
    // 读完所有的电机数据包
    ///TODO: improve: motor_hw can't know which message is which motor's info, so we can't use the 
    //                state interfaces' order to specify each motor
    for (int i = 0; i < hw_states_.size(); i++) {
        serial_->read(read_buffer_, 1);
        while (read_buffer_[0] != 0xA2) {
            serial_->read(read_buffer_, 1);
        }
        serial_->read(read_buffer_ + 1, READ_BUFFER_SIZE - 1);
        
        if (Resolver::verify_crc_check_sum(read_buffer_) && read_buffer_[13] == 0xA3) {
            Resolver::read_package_resolve(hw_states_[i], read_buffer_);
            // Resolver::read_packet_to_hw_states(read_packet_[i], hw_states_[i]);
        }
    }   
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // // show all hw commands
    // for (int i = 0; i < hw_commands_.size(); i++) {
    //     for (int j = 0; j < 4; j++) {
    //         RCLCPP_INFO(logger_, "hw_commands_[%d].cmds[%d] = %f", i, j, hw_commands_[i].cmds[j]);
    //     }
    // }
    // write commands packet
    Resolver::generate_write_packet(write_packet_, hw_commands_);
    for (int i = 0; i < write_packet_.size(); i++) {
        // show all write packages
        // for (int j = 0; j < 6; j++)
        //     RCLCPP_WARN(logger_, "write_packet_[%d].can_id = %f", i, write_packet_[i].cmds[j]);
        Resolver::write_package_resolve(write_packet_[i], write_buffer_, i);
    }
    serial_->write(write_buffer_, WRITE_BUFFER_SIZE * write_packet_.size());
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int i = 0; i < hw_states_.size(); i++) {
        for (int j = 0; j < 7; j++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, STATE_NAMES[j], &hw_states_[i].states[j]
            ));
        }
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (int i = 0; i < hw_commands_.size(); i++) {
        for (int j = 0; j < 5; j++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, COMMAND_NAMES[j], &hw_commands_[i].cmds[j]
            ));
        }
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn MotorHardware::on_error(const rclcpp_lifecycle::State & previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}


} // helios_control


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  helios_control::MotorHardware,
  hardware_interface::SystemInterface)