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

hardware_interface::CallbackReturn IMUHardware::on_init(const hardware_interface::HardwareInfo & info) {
    // init info
    if (hardware_interface::SensorInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    // resize states
    imu_packets_.resize(info_.sensors.size(), std::numeric_limits<IMUPacket>::quiet_NaN());
    std::for_each(imu_packets_.begin(), imu_packets_.end(), [this](IMUPacket& packet)->void {
        packet.state = IMU_MODE;
    });
    memset(read_buffer_, 0, sizeof(read_buffer_));
    // create serial
    serial_ = std::make_shared<serial::Serial>();
    if (!serial_) {
        RCLCPP_ERROR(logger_, "Unable to create a serial");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn IMUHardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
    try {
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

hardware_interface::CallbackReturn IMUHardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
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

hardware_interface::CallbackReturn IMUHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
    serial_->close();
    if (serial_->isOpen()) {
        RCLCPP_ERROR(logger_, "Unable to close serial");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
    serial_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    serial_->read(read_buffer_, 1);
    if (read_buffer_[0] == 0xAA) {
        serial_->read(read_buffer_ + 1, 1);
        if (read_buffer_[1] == 0xAA) {
            serial_->read(read_buffer_ + 2, 17);
            if (Resolver::verify_check_sum(read_buffer_)) {
                
            }
        }
    }
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> IMUHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
        for (int i = 0; i < imu_packets_.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, X, &imu_packets_[i].x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, Y, &imu_packets_[i].y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, Z, &imu_packets_[i].z));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, W, &imu_packets_[i].w));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, X_LINEAR_ACCEL, &imu_packets_[i].x_linear_accel));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, Y_LINEAR_ACCEL, &imu_packets_[i].y_linear_accel));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, Z_LINEAR_ACCEL, &imu_packets_[i].z_linear_accel));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, X_ANGULAR_ACCEL, &imu_packets_[i].x_angular_accel));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, Y_ANGULAR_ACCEL, &imu_packets_[i].y_angular_accel));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, Z_ANGULAR_ACCEL, &imu_packets_[i].z_angular_accel));
        }
    return state_interfaces;
}

hardware_interface::CallbackReturn IMUHardware::on_error(const rclcpp_lifecycle::State & previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace helios_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  helios_control::IMUHardware,
  hardware_interface::SensorInterface)