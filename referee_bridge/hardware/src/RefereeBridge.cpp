#include "RefereeBridge.hpp"

namespace helios_control {
    hardware_interface::CallbackReturn RefereeBridge::on_init(const hardware_interface::HardwareInfo & info) {
        RCLCPP_INFO(logger_, "on_init");
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        serial_port_ = std::make_unique<serial::Serial>();
        header_receive_buffer_ = std::make_unique<FrameBuffer>();
        serial_port_name_ = SERIAL_PORT_NAME;
        serial_port_baudrate_ = SERIAL_PORT_BAUDRATE;
        serial_port_timeout_ = SERIAL_PORT_TIMEOUT;
        if (serial_port_ == nullptr || header_receive_buffer_ == nullptr) {
            RCLCPP_ERROR(logger_, "Unable to allocate memory for serial port or frame buffer");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_configure(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(logger_, "on_configure");
        try {
            RCLCPP_INFO(logger_, "Openning serial port %s", serial_port_name_.c_str());
            serial_port_->setPort(serial_port_name_);
            serial_port_->setBaudrate(serial_port_baudrate_);
            serial_port_->setFlowcontrol(serial::flowcontrol_none);
            serial_port_->setParity(serial::parity_none); // default is parity_none
            serial_port_->setStopbits(serial::stopbits_one);
            serial_port_->setBytesize(serial::eightbits);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(serial_port_timeout_);
            serial_port_->setTimeout(timeout);
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Unable to initialize port: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Register Callbacks
        cmd_callback_map_[0x003] = std::bind(&RefereeBridge::GameRobotHPCallback, this, std::placeholders::_1, std::placeholders::_2);
        cmd_callback_map_[0x202] = std::bind(&RefereeBridge::PowerHeatDataCallback, this, std::placeholders::_1, std::placeholders::_2);
        cmd_callback_map_[0x203] = std::bind(&RefereeBridge::ShootDataCallback, this, std::placeholders::_1, std::placeholders::_2);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_activate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(logger_, "on_activate");
        try {
            serial_port_->open();
        } catch(serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Unable to open port: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (serial_port_->isOpen()) {
            RCLCPP_INFO(logger_, "Serial Port Activated");
        } else {
            RCLCPP_ERROR(logger_, "Serial Port Failed Activating");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(logger_, "on_deactivate");
        try {
            serial_port_->close();
        } catch(serial::IOException& e) {
            RCLCPP_ERROR(logger_, "Unable to close port: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!serial_port_->isOpen()) {
            RCLCPP_INFO(logger_, "Serial Port Deactivated");
        } else {
            RCLCPP_ERROR(logger_, "Serial Port Failed Deactivating");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RefereeBridge::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(logger_, "on_cleanup");
        serial_port_.reset();
        header_receive_buffer_.reset();
        if (serial_port_ != nullptr || header_receive_buffer_ != nullptr) {
            RCLCPP_ERROR(logger_, "Unable to deallocate memory for serial port or frame buffer");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RefereeBridge::read(const rclcpp::Time & , const rclcpp::Duration & )
    {
        if (!serial_port_->isOpen()) {
            RCLCPP_ERROR(logger_, "Serial port is not open");
            return hardware_interface::return_type::ERROR;
        }
        while (!(serial_port_->read(&header_receive_buffer_->sof, 1) == 1 && header_receive_buffer_->sof == SOF)) {
            RCLCPP_INFO(logger_, "Waiting for SOF");
        }
        try {
            serial_port_->read((uint8_t *)&header_receive_buffer_->data_length, 6);
            if (header_receive_buffer_->data_length > 256) {
                RCLCPP_ERROR(logger_, "Data length is too long");
                return hardware_interface::return_type::ERROR;
            }
            serial_port_->read(header_receive_buffer_->data, header_receive_buffer_->data_length + 2);
            if (header_receive_buffer_->CheckTail() != 0xa6a7 && !header_receive_buffer_->CheckPackCRC16()) {
                RCLCPP_ERROR(logger_, "Data CRC check failed");
                return hardware_interface::return_type::ERROR;
            }
        } catch(serial::SerialException& e) {
            RCLCPP_ERROR(logger_, "Unable to read from port: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        auto it = cmd_callback_map_.find(header_receive_buffer_->cmd_id);
        if (it != cmd_callback_map_.end()) {
            it->second(header_receive_buffer_->data, header_receive_buffer_->data_length);
        } else {
            RCLCPP_ERROR(logger_, "Unknown cmd_id: %d", header_receive_buffer_->cmd_id);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RefereeBridge::write(const rclcpp::Time & , const rclcpp::Duration & )
    {

        return hardware_interface::return_type::OK;
    }

    /// Not Truly Safe. Considering to rewrite.
    std::vector<hardware_interface::StateInterface> RefereeBridge::export_state_interfaces() {
        RCLCPP_INFO(logger_, "export_state_interfaces");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        // export by the order: 
        // RED1 RED2 RED3 RED4 RED5 RED7 REDO REDB
        // BLUE1 BLUE2 BLUE3 BLUE4 BLUE5 BLUE7 BLUEO BLUEB
        state_interfaces.emplace_back(hardware_interface::StateInterface(
           info_.sensors[0].name, "referee_bridge", game_robot_hp_.data()
        ));
        // export by the order:
        // chassis_volt chassis_current chassis_power chassis_power_buffer
        // shooter_id1_17mm_cooling_heat shooter_id2_17mm_cooling_heat shooter_id1_42mm_cooling_heat
        // shooter_id1_17mm_residual_cooling_heat shooter_id2_17mm_residual_cooling_heat shooter_id1_42mm_residual_cooling_heat 
        state_interfaces.emplace_back(hardware_interface::StateInterface(
           info_.sensors[1].name, "referee_bridge", power_heat_.data()
        ));
        // export by the order:
        // bullet_speed
        state_interfaces.emplace_back(hardware_interface::StateInterface(
           info_.sensors[2].name, "referee_bridge", shoot_.data()
        ));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RefereeBridge::export_command_interfaces() {
        RCLCPP_INFO(logger_, "export_command_interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        /*todo*/
        return std::vector<hardware_interface::CommandInterface>();
    }

    void RefereeBridge::GameRobotHPCallback(uint8_t * data, uint16_t data_length) {
        std::memcpy(&game_robot_hp_data_, 
        (header_receive_buffer_->data + header_receive_buffer_->GetHeaderLength()),
        LEN_game_robot_HP);
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_1_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_2_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_3_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_4_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_5_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_7_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_outpost_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.red_base_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_1_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_2_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_3_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_4_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_5_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_7_robot_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_outpost_HP));
        game_robot_hp_.emplace_back(static_cast<double>(game_robot_hp_data_.blue_base_HP));
    }

    void RefereeBridge::PowerHeatDataCallback(uint8_t * data, uint16_t data_length) {
        std::memcpy(&power_heat_data_, 
        (header_receive_buffer_->data + header_receive_buffer_->GetHeaderLength()), LEN_power_heat_data);
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.chassis_volt));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.chassis_current));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.chassis_power));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.chassis_power_buffer));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.shooter_id1_17mm_cooling_heat));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.shooter_id2_17mm_cooling_heat));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.shooter_id1_42mm_cooling_heat));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.shooter_id1_17mm_residual_cooling_heat));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.shooter_id2_17mm_residual_cooling_heat));
        power_heat_.emplace_back(static_cast<double>(power_heat_data_.shooter_id1_42mm_residual_cooling_heat));
    }

    void RefereeBridge::ShootDataCallback(uint8_t * data, uint16_t data_length) {
        std::memcpy(&shoot_data_, 
        (header_receive_buffer_->data + header_receive_buffer_->GetHeaderLength()),
        LEN_shoot_data);
        shoot_.emplace_back(static_cast<double>(shoot_data_.bullet_speed));
    }


} // namespace helios_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(helios_control::RefereeBridge, hardware_interface::SystemInterface)