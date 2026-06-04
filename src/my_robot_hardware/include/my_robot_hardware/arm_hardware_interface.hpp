// #ifndef ARM_HARDWARE_INTERFACE_HPP
// #define ARM_HARDWARE_INTERFACE_HPP

// #include "hardware_interface/system_interface.hpp"
// #include "my_robot_hardware/xl330_driver.hpp"

// namespace arm_hardware {

// class ArmHardwareInterface : public hardware_interface::SystemInterface
// {
// public:
//     // Lifecycle node override
//     hardware_interface::CallbackReturn
//         on_configure(const rclcpp_lifecycle::State & previous_state) override;
//     hardware_interface::CallbackReturn
//         on_activate(const rclcpp_lifecycle::State & previous_state) override;
//     hardware_interface::CallbackReturn
//         on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//     // SystemInterface override
//     hardware_interface::CallbackReturn
//         on_init(const hardware_interface::HardwareInfo & info) override;
//     hardware_interface::return_type
//         read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
//     hardware_interface::return_type
//         write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//     std::shared_ptr<XL330Driver> driver_;
//     int joint1_motor_id_;
//     int joint2_motor_id_;
//     std::string port_;

// }; // class ArmHardwareInterface

// } // namespace arm_hardware

// #endif


// #ifndef ARM_HARDWARE_INTERFACE_HPP
// #define ARM_HARDWARE_INTERFACE_HPP

// #include <memory>
// #include <string>
// #include <vector>
// #include <cmath>

// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "hardware_interface/handle.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// #include "my_robot_hardware/servo_serial_driver.hpp"

// namespace arm_hardware
// {

// class ArmHardwareInterface : public hardware_interface::SystemInterface
// {
// public:
//     hardware_interface::CallbackReturn on_init(
//         const hardware_interface::HardwareInfo & info) override;

//     hardware_interface::CallbackReturn on_configure(
//         const rclcpp_lifecycle::State & previous_state) override;

//     hardware_interface::CallbackReturn on_activate(
//         const rclcpp_lifecycle::State & previous_state) override;

//     hardware_interface::CallbackReturn on_deactivate(
//         const rclcpp_lifecycle::State & previous_state) override;

//     std::vector<hardware_interface::StateInterface>
//     export_state_interfaces() override;

//     std::vector<hardware_interface::CommandInterface>
//     export_command_interfaces() override;

//     hardware_interface::return_type read(
//         const rclcpp::Time & time,
//         const rclcpp::Duration & period) override;

//     hardware_interface::return_type write(
//         const rclcpp::Time & time,
//         const rclcpp::Duration & period) override;

// private:
//     std::shared_ptr<ServoSerialDriver> driver_;

//     std::string port_;
//     int baudrate_;

//     std::vector<std::string> joint_names_;
//     std::vector<int> servo_channels_;

//     std::vector<double> hw_commands_;
//     std::vector<double> hw_states_;
// };

// }  // namespace arm_hardware

// #endif

#ifndef MY_ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
#define MY_ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "my_robot_hardware/servo_serial_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
namespace arm_hardware
{

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    std::string port_;
    int baudrate_{115200};

    std::vector<std::string> joint_names_;
    std::vector<int> servo_channels_;

    std::vector<double> hw_states_;
    std::vector<double> hw_commands_;

    // Dùng để tránh gửi serial liên tục khi command không đổi
    std::vector<double> last_hw_commands_;
    rclcpp::Time last_write_time_;
    double serial_write_period_sec_{0.05};  // 0.05s = 20Hz max

    std::shared_ptr<ServoSerialDriver> driver_;
};

}  // namespace arm_hardware

#endif  // MY_ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_