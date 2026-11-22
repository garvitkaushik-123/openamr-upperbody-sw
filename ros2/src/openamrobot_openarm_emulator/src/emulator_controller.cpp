// SPDX-License-Identifier: MIT
#include "openamrobot_openarm_emulator/emulator_controller.hpp"

#include <algorithm>
#include <cstdio>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace openamrobot_openarm_emulator
{

using controller_interface::CallbackReturn;
using openarm_emulator::ArmEmulator;
using openarm_emulator::ArmEmulatorConfig;
using openarm_emulator::MotorConfig;
using openarm_emulator::MotorStatus;

namespace
{
constexpr size_t kMotors = 8;

std::string hex(uint32_t value)
{
  char text[16];
  std::snprintf(text, sizeof(text), "0x%02X", value);
  return text;
}
}  // namespace

CallbackReturn OpenArmCanEmulator::on_init()
{
  auto node = get_node();
  // Fixed while the controller is configured.
  node->declare_parameter<std::string>("can_interface", "vcan_arm_l");
  node->declare_parameter<std::string>("arm_prefix", "left_");
  node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  node->declare_parameter<std::vector<std::string>>("motor_types", std::vector<std::string>{});
  node->declare_parameter<std::vector<double>>("peak_torque", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("no_load_speed_24v", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("damping_inertia", std::vector<double>{});
  node->declare_parameter<bool>("reply_brs", true);
  node->declare_parameter<bool>("motor.answers_when_disabled", true);
  node->declare_parameter<bool>("motor.power_up_enabled", false);
  node->declare_parameter<bool>("motor.enable_clears_fault", false);
  node->declare_parameter<double>("motor.can_timeout_s", 0.0);
  node->declare_parameter<double>("motor.over_voltage_v", 32.0);
  node->declare_parameter<double>("motor.under_voltage_v", 15.0);
  node->declare_parameter<double>("motor.over_temperature_c", 120.0);
  node->declare_parameter<bool>("motor.voltage_speed_model", true);
  node->declare_parameter<double>("motor.ambient_c", 25.0);
  node->declare_parameter<bool>("motor.implicit_damping", true);
  node->declare_parameter<double>("motor.damping_split_hz", 200.0);
  // May be changed while running.
  node->declare_parameter<bool>("timing.model", true);
  node->declare_parameter<double>("timing.nominal_bitrate", 1.0e6);
  node->declare_parameter<double>("timing.data_bitrate", 5.0e6);
  node->declare_parameter<double>("timing.turnaround_us", 0.0);
  node->declare_parameter<double>("supply_voltage", 25.6);
  node->declare_parameter<bool>("fault.can_loss", false);
  node->declare_parameter<bool>("fault.estop_power_cut", false);
  node->declare_parameter<std::vector<int64_t>>("fault.silent_motors", std::vector<int64_t>(kMotors, 0));
  node->declare_parameter<std::vector<int64_t>>("fault.error_codes", std::vector<int64_t>(kMotors, 0));
  node->declare_parameter<std::vector<double>>("fault.temperature_mos_c", std::vector<double>(kMotors, 25.0));
  node->declare_parameter<std::vector<double>>("fault.temperature_rotor_c", std::vector<double>(kMotors, 25.0));
  node->declare_parameter<double>("fault.temperature_rate_c_per_s", 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmCanEmulator::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  ArmEmulatorConfig config;
  config.can_interface = node->get_parameter("can_interface").as_string();
  config.motors = openarm_emulator::default_arm_motors(node->get_parameter("arm_prefix").as_string());
  const auto joints = node->get_parameter("joints").as_string_array();
  const auto types = node->get_parameter("motor_types").as_string_array();
  const auto peak = node->get_parameter("peak_torque").as_double_array();
  const auto speed = node->get_parameter("no_load_speed_24v").as_double_array();
  const auto inertia = node->get_parameter("damping_inertia").as_double_array();
  for (const auto & [name, size] : {std::pair<const char *, size_t>{"joints", joints.size()}, {"motor_types", types.size()},
      {"peak_torque", peak.size()}, {"no_load_speed_24v", speed.size()}, {"damping_inertia", inertia.size()}})
  {
    if (size != 0 && size != kMotors) {
      RCLCPP_ERROR(node->get_logger(), "%s has %zu entries, expected 0 (defaults) or %zu", name, size, kMotors);
      return CallbackReturn::ERROR;
    }
  }
  for (size_t i = 0; i < kMotors; ++i) {
    MotorConfig & m = config.motors[i];
    if (!joints.empty()) {m.joint = joints[i];}
    if (!types.empty() && !openarm_emulator::motor_type_from_name(types[i].c_str(), m.type)) {
      RCLCPP_ERROR(node->get_logger(), "motor_types[%zu] = '%s': expected DM4310, DM4340 or DM8009", i, types[i].c_str());
      return CallbackReturn::ERROR;
    }
    if (!peak.empty()) {m.peak_torque = peak[i];}
    if (!speed.empty()) {m.no_load_speed_24v = speed[i];}
    if (!inertia.empty()) {m.damping_inertia = inertia[i];}
  }
  config.reply_brs = node->get_parameter("reply_brs").as_bool();
  config.options.answers_when_disabled = node->get_parameter("motor.answers_when_disabled").as_bool();
  config.options.power_up_enabled = node->get_parameter("motor.power_up_enabled").as_bool();
  config.options.enable_clears_fault = node->get_parameter("motor.enable_clears_fault").as_bool();
  config.options.can_timeout_s = node->get_parameter("motor.can_timeout_s").as_double();
  config.options.over_voltage_v = node->get_parameter("motor.over_voltage_v").as_double();
  config.options.under_voltage_v = node->get_parameter("motor.under_voltage_v").as_double();
  config.options.over_temperature_c = node->get_parameter("motor.over_temperature_c").as_double();
  config.options.voltage_speed_model = node->get_parameter("motor.voltage_speed_model").as_bool();
  config.options.ambient_c = node->get_parameter("motor.ambient_c").as_double();
  config.options.implicit_damping = node->get_parameter("motor.implicit_damping").as_bool();
  config.options.damping_split_hz = node->get_parameter("motor.damping_split_hz").as_double();
  config.supply_voltage = node->get_parameter("supply_voltage").as_double();

  joints_.clear();
  for (const MotorConfig & m : config.motors) {joints_.push_back(m.joint);}
  q_.assign(kMotors, 0.0);
  dq_.assign(kMotors, 0.0);
  tau_.assign(kMotors, 0.0);
  emulator_ = std::make_unique<ArmEmulator>(config);
  apply_faults();

  parameter_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {return on_parameters(parameters);});
  status_publisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/status", rclcpp::QoS(10));
  // Wall time: the host side runs on wall time, and the status must keep coming when the simulation is paused.
  status_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this] {publish_status();});
  RCLCPP_INFO(node->get_logger(), "OpenArm motor emulator for %s on %s (simulation only, no safety function)",
    joints_.front().c_str(), config.can_interface.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OpenArmCanEmulator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const std::string & joint : joints_) {config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);}
  return config;
}

controller_interface::InterfaceConfiguration OpenArmCanEmulator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const std::string & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

CallbackReturn OpenArmCanEmulator::on_activate(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!emulator_->start(error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s. Create the interface first: ros2 run openamrobot_openarm_emulator setup_vcan.sh",
      error.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "answering on %s, CAN FD %s", get_node()->get_parameter("can_interface").as_string().c_str(),
    emulator_->fd_capable() ? "yes" : "NO (interface MTU is 16: classic CAN frames only; run the driver with can_fd false)");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenArmCanEmulator::on_deactivate(const rclcpp_lifecycle::State &)
{
  emulator_->stop();
  for (auto & command : command_interfaces_) {(void)command.set_value(0.0);}
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OpenArmCanEmulator::update(const rclcpp::Time &, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < kMotors; ++i) {
    q_[i] = state_interfaces_[2 * i].get_optional<double>().value_or(q_[i]);
    dq_[i] = state_interfaces_[2 * i + 1].get_optional<double>().value_or(dq_[i]);
  }
  emulator_->step(q_, dq_, period.seconds(), tau_);
  for (size_t i = 0; i < kMotors; ++i) {(void)command_interfaces_[i].set_value(tau_[i]);}
  return controller_interface::return_type::OK;
}

rcl_interfaces::msg::SetParametersResult OpenArmCanEmulator::on_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (!emulator_) {return result;}
  openarm_emulator::BusTimingConfig timing;
  auto node = get_node();
  timing.enabled = node->get_parameter("timing.model").as_bool();
  timing.nominal_bitrate = node->get_parameter("timing.nominal_bitrate").as_double();
  timing.data_bitrate = node->get_parameter("timing.data_bitrate").as_double();
  timing.turnaround_s = node->get_parameter("timing.turnaround_us").as_double() * 1e-6;
  std::vector<double> mos = node->get_parameter("fault.temperature_mos_c").as_double_array();
  std::vector<double> rotor = node->get_parameter("fault.temperature_rotor_c").as_double_array();
  double rate = node->get_parameter("fault.temperature_rate_c_per_s").as_double();
  bool timing_changed = false, temperature_changed = false;

  for (const rclcpp::Parameter & p : parameters) {
    const std::string & name = p.get_name();
    const auto wrong_size = [&](size_t size) {
        if (size == kMotors) {return false;}
        result.successful = false;
        result.reason = name + " needs " + std::to_string(kMotors) + " entries, one per motor (J1..J7, gripper)";
        return true;
      };
    if (name == "supply_voltage") {
      emulator_->set_supply_voltage(p.as_double());
    } else if (name == "fault.can_loss") {
      emulator_->set_can_loss(p.as_bool());
    } else if (name == "fault.estop_power_cut") {
      emulator_->set_power(!p.as_bool());
    } else if (name == "fault.silent_motors") {
      if (wrong_size(p.as_integer_array().size())) {return result;}
      for (size_t i = 0; i < kMotors; ++i) {emulator_->set_motor_silent(i, p.as_integer_array()[i] != 0);}
    } else if (name == "fault.error_codes") {
      if (wrong_size(p.as_integer_array().size())) {return result;}
      for (int64_t code : p.as_integer_array()) {
        if (code != 0 && (code < 0x8 || code > 0xE)) {
          result.successful = false;
          result.reason = "fault.error_codes entries are 0 (none) or 8 to 14 (0x8 over-voltage to 0xE overload)";
          return result;
        }
      }
      for (size_t i = 0; i < kMotors; ++i) {emulator_->force_code(i, static_cast<uint8_t>(p.as_integer_array()[i]));}
    } else if (name == "fault.temperature_mos_c") {
      if (wrong_size(p.as_double_array().size())) {return result;}
      mos = p.as_double_array();
      temperature_changed = true;
    } else if (name == "fault.temperature_rotor_c") {
      if (wrong_size(p.as_double_array().size())) {return result;}
      rotor = p.as_double_array();
      temperature_changed = true;
    } else if (name == "fault.temperature_rate_c_per_s") {
      rate = p.as_double();
      temperature_changed = true;
    } else if (name == "timing.model") {
      timing.enabled = p.as_bool();
      timing_changed = true;
    } else if (name == "timing.nominal_bitrate" || name == "timing.data_bitrate") {
      if (p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = name + " must be positive";
        return result;
      }
      (name == "timing.nominal_bitrate" ? timing.nominal_bitrate : timing.data_bitrate) = p.as_double();
      timing_changed = true;
    } else if (name == "timing.turnaround_us") {
      timing.turnaround_s = p.as_double() * 1e-6;
      timing_changed = true;
    }
  }
  if (timing_changed) {emulator_->set_timing(timing);}
  if (temperature_changed) {
    for (size_t i = 0; i < kMotors; ++i) {emulator_->set_temperature_target(i, mos[i], rotor[i], rate);}
  }
  return result;
}

void OpenArmCanEmulator::apply_faults()
{
  auto node = get_node();
  std::vector<rclcpp::Parameter> all;
  for (const char * name : {"supply_voltage", "fault.can_loss", "fault.estop_power_cut", "fault.silent_motors", "fault.error_codes",
      "fault.temperature_mos_c", "fault.temperature_rate_c_per_s", "timing.model"})
  {
    all.push_back(node->get_parameter(name));
  }
  const auto result = on_parameters(all);
  if (!result.successful) {RCLCPP_ERROR(node->get_logger(), "%s", result.reason.c_str());}
}

void OpenArmCanEmulator::publish_status()
{
  if (!emulator_ || !status_publisher_) {return;}
  const auto kv = [](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      return pair;
    };
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  const std::string can = get_node()->get_parameter("can_interface").as_string();

  diagnostic_msgs::msg::DiagnosticStatus bus;
  bus.name = "openarm_emulator/" + can;
  bus.hardware_id = can;
  const auto stats = emulator_->stats();
  bus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  bus.message = !emulator_->running() ? "stopped" : (!emulator_->powered() ? "E-stop power cut" : (emulator_->can_loss() ? "CAN loss" : "running"));
  bus.values = {kv("running", emulator_->running() ? "true" : "false"), kv("can_fd", emulator_->fd_capable() ? "true" : "false"),
    kv("powered", emulator_->powered() ? "true" : "false"), kv("can_loss", emulator_->can_loss() ? "true" : "false"),
    kv("supply_voltage", std::to_string(emulator_->supply_voltage())), kv("rx_frames", std::to_string(stats.rx_frames)),
    kv("rx_fd", std::to_string(stats.rx_fd)), kv("rx_classic", std::to_string(stats.rx_classic)),
    kv("rx_unknown", std::to_string(stats.rx_unknown)), kv("rx_errors", std::to_string(stats.rx_errors)),
    kv("tx_frames", std::to_string(stats.tx_frames)), kv("tx_errors", std::to_string(stats.tx_errors))};
  array.status.push_back(bus);

  for (size_t i = 0; i < emulator_->size(); ++i) {
    const MotorStatus s = emulator_->status(i);
    const MotorConfig & c = emulator_->motor_config(i);
    diagnostic_msgs::msg::DiagnosticStatus motor;
    motor.name = "openarm_emulator/" + can + "/" + c.joint;
    motor.hardware_id = std::string(openarm_emulator::motor_type_name(c.type)) + " " + hex(c.send_id) + "/" + hex(c.recv_id);
    motor.level = s.code >= openarm_emulator::kFaultThreshold ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    motor.message = !s.powered ? "no power" : (s.code >= openarm_emulator::kFaultThreshold ? "fault " + hex(s.code) :
      (s.enabled ? "enabled" : "disabled"));
    motor.values = {kv("powered", s.powered ? "true" : "false"), kv("enabled", s.enabled ? "true" : "false"),
      kv("silent", s.silent ? "true" : "false"), kv("code", std::to_string(s.code)),
      kv("mode", std::to_string(static_cast<uint32_t>(s.mode))), kv("position", std::to_string(s.q)),
      kv("velocity", std::to_string(s.dq)), kv("torque", std::to_string(s.tau)), kv("t_mos", std::to_string(s.t_mos)),
      kv("t_rotor", std::to_string(s.t_rotor)), kv("mit_q", std::to_string(s.last_mit.q)), kv("mit_kp", std::to_string(s.last_mit.kp)),
      kv("mit_kd", std::to_string(s.last_mit.kd)), kv("enable_frames", std::to_string(s.enable_frames)),
      kv("disable_frames", std::to_string(s.disable_frames)), kv("control_frames", std::to_string(s.control_frames)),
      kv("refresh_frames", std::to_string(s.refresh_frames)), kv("replies", std::to_string(s.replies)),
      kv("first_mit_q", s.first_mit_valid ? std::to_string(s.first_mit.q) : ""),
      kv("first_mit_position", s.first_mit_valid ? std::to_string(s.first_mit_position) : "")};
    array.status.push_back(motor);
  }
  status_publisher_->publish(array);
}

}  // namespace openamrobot_openarm_emulator

PLUGINLIB_EXPORT_CLASS(openamrobot_openarm_emulator::OpenArmCanEmulator, controller_interface::ControllerInterface)
