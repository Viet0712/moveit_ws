// #include "my_robot_hardware/arm_hardware_interface.hpp"

// namespace arm_hardware {

// hardware_interface::CallbackReturn ArmHardwareInterface::on_init
//     (const hardware_interface::HardwareInfo & info)
// {
//     if (hardware_interface::SystemInterface::on_init(info) !=
//         hardware_interface::CallbackReturn::SUCCESS)
//     {
//         return hardware_interface::CallbackReturn::ERROR;
//     }

//     info_ = info;

//     joint1_motor_id_ = std::stoi(info_.hardware_parameters["joint1_motor_id"]);
//     joint2_motor_id_ = std::stoi(info_.hardware_parameters["joint2_motor_id"]);
//     port_ = info_.hardware_parameters["dynamixel_port"];

//     driver_ = std::make_shared<XL330Driver>(port_);

//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn ArmHardwareInterface::on_configure
//     (const rclcpp_lifecycle::State & previous_state)
// {
//     (void)previous_state;
//     if (driver_->init() !=0) {
//         return hardware_interface::CallbackReturn::ERROR;
//     }
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn ArmHardwareInterface::on_activate
//     (const rclcpp_lifecycle::State & previous_state)
// {
//     (void)previous_state;

//     set_state("joint1/position", driver_->getPositionRadian(joint1_motor_id_));
//     set_state("joint2/position", driver_->getPositionRadian(joint2_motor_id_));
//     set_state("joint3/position", 0.0);
//     set_state("joint4/position", 0.0);
//     set_state("joint5/position", 0.0);
//     set_state("joint6/position", 0.0);

//     driver_->activateWithPositionMode(joint1_motor_id_);
//     driver_->activateWithPositionMode(joint2_motor_id_);
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
//     (const rclcpp_lifecycle::State & previous_state)
// {
//     (void)previous_state;
//     driver_->deactivate(joint1_motor_id_);
//     driver_->deactivate(joint2_motor_id_);
//     return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type ArmHardwareInterface::read
//     (const rclcpp::Time & time, const rclcpp::Duration & period)
// {
//     (void)time;
//     (void)period;

//     set_state("joint1/position", driver_->getPositionRadian(joint1_motor_id_));
//     set_state("joint2/position", driver_->getPositionRadian(joint2_motor_id_));

//     // RCLCPP_INFO(get_logger(), "STATE joint1: %lf, joint2: %lf", 
//     //     get_state("joint1/position"), get_state("joint2/position"));

//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type ArmHardwareInterface::write
//     (const rclcpp::Time & time, const rclcpp::Duration & period)
// {
//     (void)time;
//     (void)period;
    
//     if (!std::isnan(get_command("joint1/position"))) {
//         driver_->setTargetPositionRadian(joint1_motor_id_, get_command("joint1/position"));
//         driver_->setTargetPositionRadian(joint2_motor_id_, get_command("joint2/position"));

//         // Fake hardware for motors 3-6
//         set_state("joint3/position", get_command("joint3/position"));
//         set_state("joint4/position", get_command("joint4/position"));
//         set_state("joint5/position", get_command("joint5/position"));
//         set_state("joint6/position", get_command("joint6/position"));

//         // RCLCPP_INFO(get_logger(), "COMMAND joint1: %lf, joint2: %lf", 
//         //     get_command("joint1/position"), get_command("joint2/position"));
//     }

//     return hardware_interface::return_type::OK;
// }

// } // namespace arm_hardware

// #include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)




#include "my_robot_hardware/arm_hardware_interface.hpp"

namespace arm_hardware
{

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    port_ = info_.hardware_parameters["serial_port"];
    baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

    joint_names_ = {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper_left_finger_joint"
    };

    servo_channels_.resize(joint_names_.size());

    servo_channels_[0] = std::stoi(info_.hardware_parameters["joint1_channel"]);
    servo_channels_[1] = std::stoi(info_.hardware_parameters["joint2_channel"]);
    servo_channels_[2] = std::stoi(info_.hardware_parameters["joint3_channel"]);
    servo_channels_[3] = std::stoi(info_.hardware_parameters["joint4_channel"]);
    servo_channels_[4] = std::stoi(info_.hardware_parameters["joint5_channel"]);
    servo_channels_[5] = std::stoi(info_.hardware_parameters["joint6_channel"]);
    servo_channels_[6] = std::stoi(info_.hardware_parameters["gripper_channel"]);

    hw_commands_.resize(joint_names_.size(), 0.0);
    hw_states_.resize(joint_names_.size(), 0.0);

    driver_ = std::make_shared<ServoSerialDriver>(port_, baudrate_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (driver_->init() != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ArmHardwareInterface"),
            "Failed to initialize ServoSerialDriver"
        );
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("ArmHardwareInterface"),
        "ServoSerialDriver initialized successfully"
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        hw_commands_[i] = 0.0;
        hw_states_[i] = 0.0;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Arm hardware activated"
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (size_t i = 0; i < servo_channels_.size(); ++i) {
        driver_->deactivate(servo_channels_[i]);
    }

    RCLCPP_INFO(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Arm hardware deactivated"
    );

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &hw_states_[i]
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &hw_commands_[i]
            )
        );
    }

    return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    /*
     * Nếu chưa có encoder:
     * driver_->getMeasuredPositionRadian() đang trả command cuối cùng.
     *
     * Nếu sau này ESP32 có encoder:
     * sửa ServoSerialDriver để đọc feedback thật từ ESP32.
     */

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        hw_states_[i] = driver_->getMeasuredPositionRadian(servo_channels_[i]);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (!std::isnan(hw_commands_[i])) {
            driver_->setTargetPositionRadian(
                servo_channels_[i],
                hw_commands_[i]
            );
        }
    }

    return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware::ArmHardwareInterface,
    hardware_interface::SystemInterface
)