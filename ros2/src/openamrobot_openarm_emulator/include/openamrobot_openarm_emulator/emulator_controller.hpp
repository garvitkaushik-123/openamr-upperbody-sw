// SPDX-License-Identifier: MIT
// The emulator of one arm as a ros2_control controller. It runs in the controller manager that gz_ros2_control hosts
// inside the simulator: every simulation step it reads the eight joints' position and velocity, runs the emulated
// motors' control laws and commands the joints' effort. On the other side it answers the host on a SocketCAN interface.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "openamrobot_openarm_emulator/arm_emulator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openamrobot_openarm_emulator
{

class OpenArmCanEmulator : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);
  void apply_faults();
  void publish_status();

  std::unique_ptr<openarm_emulator::ArmEmulator> emulator_;
  std::vector<std::string> joints_;
  std::vector<double> q_, dq_, tau_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_handle_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace openamrobot_openarm_emulator
