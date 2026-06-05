
#include "my_robot_hardware/arm_hardware_interface.hpp"

namespace arm_hardware
{

    hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            port_ = info.hardware_parameters.at("serial_port");
            baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));

            joint_names_ = {
                "gripper_left_finger_joint",
                "joint1",
                "joint2",
                "joint3",
            };

            servo_channels_.resize(joint_names_.size());
            servo_channels_[1] = std::stoi(info.hardware_parameters.at("joint1_channel"));
            servo_channels_[2] = std::stoi(info.hardware_parameters.at("joint2_channel"));
            servo_channels_[3] = std::stoi(info.hardware_parameters.at("joint3_channel"));
            servo_channels_[0] = std::stoi(info.hardware_parameters.at("gripper_channel"));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("ArmHardwareInterface"),
                "Failed to read hardware parameters: %s",
                e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_states_.resize(joint_names_.size(), 0.0);
        hw_commands_.resize(joint_names_.size(), 0.0);

        last_hw_commands_.resize(
            joint_names_.size(),
            std::numeric_limits<double>::quiet_NaN());

        driver_ = std::make_shared<ServoSerialDriver>(port_, baudrate_);

        RCLCPP_INFO(
            rclcpp::get_logger("ArmHardwareInterface"),
            "ArmHardwareInterface initialized with %zu joints",
            joint_names_.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        if (!driver_)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("ArmHardwareInterface"),
                "ServoSerialDriver is null");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (driver_->init() != 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("ArmHardwareInterface"),
                "Failed to initialize ServoSerialDriver");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("ArmHardwareInterface"),
            "ServoSerialDriver initialized successfully");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            hw_states_[i] = 0.0;
            hw_commands_[i] = hw_states_[i];

            // Để write() lần đầu vẫn gửi command nếu cần
            last_hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
        }

        RCLCPP_INFO(
            rclcpp::get_logger("ArmHardwareInterface"),
            "Arm hardware activated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        if (driver_)
        {
            for (size_t i = 0; i < servo_channels_.size(); ++i)
            {
                driver_->deactivate(servo_channels_[i]);
            }
        }

        RCLCPP_INFO(
            rclcpp::get_logger("ArmHardwareInterface"),
            "Arm hardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    ArmHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            state_interfaces.emplace_back(
                joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &hw_states_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    ArmHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            command_interfaces.emplace_back(
                joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &hw_commands_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::return_type ArmHardwareInterface::read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        /*
         * Servo SG90 / MG995 / PCA9685 thường không có feedback vị trí thật.
         * Không nên đọc serial liên tục trong read(), vì sẽ làm nghẽn controller_manager.
         *
         * Tạm thời cho state bám theo command.
         * MoveIt và joint_state_broadcaster sẽ thấy robot đang ở vị trí đã command.
         */
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (!std::isnan(hw_commands_[i]))
            {
                hw_states_[i] = hw_commands_[i];
            }
        }

        return hardware_interface::return_type::OK;
    }

    // hardware_interface::return_type ArmHardwareInterface::write(
    //     const rclcpp::Time & time,
    //     const rclcpp::Duration & period)
    // {
    //     (void)time;
    //     (void)period;

    //     if (!driver_) {
    //         return hardware_interface::return_type::ERROR;
    //     }

    //     constexpr double EPS = 1e-4;

    //     for (size_t i = 0; i < joint_names_.size(); ++i) {
    //         if (std::isnan(hw_commands_[i])) {
    //             continue;
    //         }

    //         const bool first_command = std::isnan(last_hw_commands_[i]);
    //         const bool command_changed =
    //             std::fabs(hw_commands_[i] - last_hw_commands_[i]) > EPS;

    //         if (first_command || command_changed) {
    //             driver_->setTargetPositionRadian(
    //                 servo_channels_[i],
    //                 hw_commands_[i]
    //             );

    //             last_hw_commands_[i] = hw_commands_[i];
    //         }
    //     }

    //     return hardware_interface::return_type::OK;
    // }
    hardware_interface::return_type ArmHardwareInterface::write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period)
    {
        (void)period;

        if (!driver_)
        {
            return hardware_interface::return_type::ERROR;
        }

        // Giới hạn tần suất gửi serial
        if (last_write_time_.nanoseconds() != 0)
        {
            double dt = (time - last_write_time_).seconds();

            if (dt < serial_write_period_sec_)
            {
                return hardware_interface::return_type::OK;
            }
        }

        last_write_time_ = time;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (std::isnan(hw_commands_[i]))
            {
                continue;
            }

            // EPS riêng cho từng loại joint
            double eps = 0.02; // rad cho joint quay

            if (joint_names_[i] == "gripper_left_finger_joint")
            {
                eps = 0.0005; // m, tức 0.5 mm cho gripper
            }

            const bool first_command = std::isnan(last_hw_commands_[i]);
            const double diff = std::fabs(hw_commands_[i] - last_hw_commands_[i]);
            const bool command_changed = diff > eps;

            // RCLCPP_INFO(
            //     rclcpp::get_logger("ArmHardwareInterface"),
            //     "[CHECK] joint=%s cmd=%.6f last=%.6f diff=%.6f eps=%.6f first=%d changed=%d",
            //     joint_names_[i].c_str(),
            //     hw_commands_[i],
            //     last_hw_commands_[i],
            //     diff,
            //     eps,
            //     first_command,
            //     command_changed);

            if (first_command || command_changed)
            {
                if (joint_names_[i] == "gripper_left_finger_joint")
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger("ArmHardwareInterface"),
                        "[WRITE][GRIPPER] joint=%s channel=%d command_m=%.6f last=%.6f",
                        joint_names_[i].c_str(),
                        servo_channels_[i],
                        hw_commands_[i],
                        last_hw_commands_[i]);

                    driver_->setGripperPositionMeter(
                        servo_channels_[i],
                        hw_commands_[i]);
                }
                else
                {
                    RCLCPP_INFO(
                        rclcpp::get_logger("ArmHardwareInterface"),
                        "[WRITE][ARM] joint=%s channel=%d command_rad=%.6f last=%.6f",
                        joint_names_[i].c_str(),
                        servo_channels_[i],
                        hw_commands_[i],
                        last_hw_commands_[i]);

                    driver_->setTargetPositionRadian(
                        servo_channels_[i],
                        hw_commands_[i]);
                }

                last_hw_commands_[i] = hw_commands_[i];
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware::ArmHardwareInterface,
    hardware_interface::SystemInterface)