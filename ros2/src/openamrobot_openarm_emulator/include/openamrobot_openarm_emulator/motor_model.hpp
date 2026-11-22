// SPDX-License-Identifier: MIT
// One emulated Damiao motor: what it does with each frame the host sends and what torque it applies.
// Pure logic. No socket, no clock of its own, no ROS. Times are seconds on a monotonic clock the caller owns.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "openamrobot_openarm_emulator/codec.hpp"

namespace openarm_emulator
{

enum class ControlMode : uint32_t { MIT = 1, POS_VEL = 2, VEL = 3, POS_FORCE = 4 };

struct MotorConfig
{
  std::string joint;          // ros2_control joint this motor drives
  MotorType type{MotorType::DM4310};
  uint32_t send_id{1};        // host to motor
  uint32_t recv_id{0x11};     // motor to host
  double peak_torque{7.0};    // N m at the output. The motor never applies more.
  double no_load_speed_24v{20.944};  // rad/s at the output, at 24 V
  double damping_inertia{1.0e-4};    // kg m^2: the child link (with what is fixed to it) about the joint axis
};

/// Behaviour that no source in scope fixes. Each default is an assumption, listed in the package README.
struct MotorOptions
{
  bool answers_when_disabled{true};   // a disabled motor still answers refresh and control frames
  bool power_up_enabled{false};       // state after supply returns
  bool enable_clears_fault{false};    // false: only a clear-error frame (..FB) or a power cycle clears a fault
  double can_timeout_s{0.0};          // 0: off. Else: enabled and no control frame for this long gives code 0xD
  double over_voltage_v{32.0};        // supply above this gives 0x8
  double under_voltage_v{15.0};       // supply below this gives 0x9
  double over_temperature_c{120.0};   // MOS above this gives 0xB, rotor above this gives 0xC
  bool voltage_speed_model{true};     // no-load speed scales with supply / 24 V, torque falls linearly to 0 there
  double ambient_c{25.0};
  double posvel_kp{20.0};             // 1/s, position error to speed reference (POS_VEL and POS_FORCE)
  double vel_kd_scale{1.0};           // speed loop gain = vel_kd_scale * peak_torque / (0.1 * no_load_speed)
  bool reply_to_set_zero{true};
  // Numerical, not physical. README, "Damping and the physics step". The damping gain of the control law is too
  // stiff for an explicit 1 ms physics step on the light links (kd * dt / link inertia is up to 6; above 2 the joint
  // chatters at torque saturation). With this on, the part of the law's torque above damping_split_hz is divided by
  // 1 + kd * dt / (safety * damping_inertia), as an implicit step of the damping term would. Torque at rest and
  // motion below the split frequency are the law's own.
  bool implicit_damping{true};
  double damping_split_hz{200.0};
  double damping_inertia_safety{0.5};   // two neighbouring joints can damp the same link
};

enum class ReplyKind { None, State, Param };

struct Reply
{
  ReplyKind kind{ReplyKind::None};
  Payload param{};  // payload when kind == Param. A State payload is built when the frame is sent.
};

struct MotorStatus
{
  bool powered{true};
  bool enabled{false};
  bool silent{false};
  uint8_t code{0};
  ControlMode mode{ControlMode::MIT};
  double q{0.0};
  double dq{0.0};
  double tau{0.0};
  double t_mos{25.0};
  double t_rotor{25.0};
  MitCommand last_mit{};
  uint64_t control_frames{0};
  uint64_t enable_frames{0};
  uint64_t disable_frames{0};
  uint64_t refresh_frames{0};
  uint64_t param_frames{0};
  uint64_t replies{0};
  // The first MIT frame after the latest enable, and the motor position when it arrived.
  bool first_mit_valid{false};
  MitCommand first_mit{};
  double first_mit_position{0.0};
};

class MotorModel
{
public:
  MotorModel(const MotorConfig & config, const MotorOptions & options);

  const MotorConfig & config() const {return config_;}
  EncodingLimits limits() const {return limits_;}

  /// A frame on the motor's own id (send_id, or send_id + 0x100, 0x200, 0x300; `offset` says which).
  Reply on_control_frame(uint32_t offset, const Payload & data, double now);
  /// A frame on 0x7FF addressed to this motor.
  Reply on_config_frame(const ConfigFrame & frame, double now);

  /// One physics step. q and dq are the joint's. Returns the torque to apply to the joint, N m.
  double step(double q, double dq, double now, double dt, double supply_v);

  /// The state frame as it is right now.
  Payload state_frame();

  // Fault and scenario inputs.
  void set_power(bool on, double now);      // false: silent, no torque. true: power-up state.
  void set_silent(bool silent) {status_.silent = silent;}
  void force_code(uint8_t code);            // 0x8..0xE: fault. 0: remove a forced fault.
  void set_temperature_target(double mos_c, double rotor_c, double rate_c_per_s);

  bool replies() const {return status_.powered && !status_.silent;}
  const MotorStatus & status() const {return status_;}
  uint32_t param_raw(uint8_t rid) const;

private:
  void trip(uint8_t code);
  void clear_setpoints();
  /// The control law as tau = stiff - damping * dq. Returns {stiff, damping}.
  std::pair<double, double> law(double q) const;

  MotorConfig config_;
  MotorOptions options_;
  EncodingLimits limits_;
  MotorStatus status_;
  std::map<uint8_t, uint32_t> ram_;
  std::map<uint8_t, uint32_t> flash_;
  double zero_offset_{0.0};
  double joint_q_{0.0};
  double last_control_time_{0.0};
  uint8_t forced_code_{0};
  PosVelCommand posvel_{};
  double vel_{0.0};
  PosForceCommand posforce_{};
  double target_mos_{25.0};
  double target_rotor_{25.0};
  double temperature_rate_{0.0};
  double tau_slow_{0.0};   // applied torque, low-passed (implicit_damping)
};

}  // namespace openarm_emulator
