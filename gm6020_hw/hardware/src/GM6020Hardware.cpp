#include "GM6020Hardware.hpp"
#include <hardware_interface/actuator_interface.hpp>
#include <rclcpp/logging.hpp>
#include <serial/serial.h>

namespace helios_control {
    hardware_interface::CallbackReturn GM6020Hardware::on_init(const hardware_interface::HardwareInfo & info) {
        // init info
        if (hardware_interface::ActuatorInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // resize states
        hw_states_.resize(info_.joints.size());
        // check the number of joints
        if (info.joints.size() > 4) {
            RCLCPP_ERROR(logger_, "The number of actuators should be less than 4");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // // check the number of parameters
        // if (info.hardware_parameters.size() != 1) {
        //     RCLCPP_ERROR(logger_, "need the name of serial");
        //     return hardware_interface::CallbackReturn::ERROR;
        // }
        // create serial
        serial_ = std::make_shared<serial::Serial>();
        if (!serial_) {
            RCLCPP_ERROR(logger_, "Unable to create a serial");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
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

    hardware_interface::CallbackReturn GM6020Hardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
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
        hw_command_.actuator_current_1 = 0;
        hw_command_.actuator_current_2 = 0;
        hw_command_.actuator_current_3 = 0;
        hw_command_.actuator_current_4 = 0;
        for (auto &state : hw_states_) {
            state.angle = 0;
            state.speed = 0;
            state.effort = 0;
            state.temperature = 0;
        }
        RCLCPP_INFO(logger_, "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        serial_->close();
        if (serial_->isOpen()) {
            RCLCPP_ERROR(logger_, "Unable to close serial");
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GM6020Hardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) {
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GM6020Hardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces) {
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
        serial_.reset();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GM6020Hardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        
        serial_->read(read_buffer_, 1);
        if (read_buffer_[0] == 0x02) {
            serial_->read(read_buffer_ + 1, 1);
            int sign = read_buffer_[0] << 8;
            sign = sign | read_buffer_[1];
            if (sign == 0x201 || sign == 0x206 || sign == 0x207 || sign == 0x208) {
            // if (sign == 0x205 || sign == 0x206 || sign == 0x207 || sign == 0x208) {
                serial_->read(read_buffer_ + 2, 8);
            }
        }        
        convert_read_buffer_to_states(read_buffer_, hw_states_);
        for (int i = 0; i < hw_states_.size(); i++) {
            hw_states_[i].states[0] = hw_states_[i].angle;
            hw_states_[i].states[1] = hw_states_[i].speed;
            hw_states_[i].states[2] = hw_states_[i].effort;
            hw_states_[i].states[3] = hw_states_[i].temperature;
        }   
        return hardware_interface::return_type::OK; 
    }

    hardware_interface::return_type GM6020Hardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        // 
        // RCLCPP_INFO(logger_, "Angle: %d, Speed: %d, Effort: %d, Temperature: %d", hw_states_[1].angle, hw_states_[1].speed, hw_states_[1].effort, hw_states_[1].temperature);
        RCLCPP_INFO(logger_, "Angle: %d, Speed: %d, Effort: %d, Temperature: %d", hw_states_[0].angle, hw_states_[0].speed, hw_states_[0].effort, hw_states_[0].temperature);
        convert_command_to_write_buffer(hw_command_, write_buffer);
        try {
            serial_->write(write_buffer, 10);
        } catch (serial::SerialException& e) {
            RCLCPP_FATAL(logger_, "Fail to write commands: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> GM6020Hardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (int i = 0; i < hw_states_.size(); i++) {
            hw_states_[i].states[0] = hw_states_[i].angle;
            hw_states_[i].states[1] = hw_states_[i].speed;
            hw_states_[i].states[2] = hw_states_[i].effort;
            hw_states_[i].states[3] = hw_states_[i].temperature;
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "angle", &hw_states_[i].states[0]
            ));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "speed", &hw_states_[i].states[1]
            ));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "effort", &hw_states_[i].states[2]
            ));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "temperature", &hw_states_[i].states[3]
            ));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> GM6020Hardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (int i = 0; i < 4; i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "current", &hw_command_.cmds[i]
            ));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn GM6020Hardware::on_error(const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace helios_control


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  helios_control::GM6020Hardware,
  hardware_interface::ActuatorInterface)
