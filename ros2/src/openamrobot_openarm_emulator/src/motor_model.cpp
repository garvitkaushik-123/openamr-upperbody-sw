// SPDX-License-Identifier: MIT
#include "openamrobot_openarm_emulator/motor_model.hpp"

#include <algorithm>
#include <cmath>

namespace openarm_emulator
{

namespace
{
double clamp(double x, double lo, double hi) {return std::max(lo, std::min(x, hi));}

double approach(double value, double target, double rate, double dt)
{
  if (rate <= 0.0) {return target;}
  const double step = rate * dt;
  if (std::fabs(target - value) <= step) {return target;}
  return value + (target > value ? step : -step);
}
}  // namespace

MotorModel::MotorModel(const MotorConfig & config, const MotorOptions & options)
: config_(config), options_(options), limits_(encoding_limits(config.type))
{
  flash_[kRidUnderVoltage] = float_bits(static_cast<float>(options.under_voltage_v));
  flash_[kRidOverTemperature] = float_bits(static_cast<float>(options.over_temperature_c));
  flash_[kRidMasterId] = config.recv_id;
  flash_[kRidSlaveId] = config.send_id;
  flash_[kRidTimeout] = 0;
  flash_[kRidControlMode] = static_cast<uint32_t>(ControlMode::MIT);
  flash_[kRidPMax] = float_bits(static_cast<float>(limits_.p_max));
  flash_[kRidVMax] = float_bits(static_cast<float>(limits_.v_max));
  flash_[kRidTMax] = float_bits(static_cast<float>(limits_.t_max));
  flash_[kRidOverVoltage] = float_bits(static_cast<float>(options.over_voltage_v));
  ram_ = flash_;
  status_.t_mos = status_.t_rotor = target_mos_ = target_rotor_ = options.ambient_c;
  status_.enabled = options.power_up_enabled;
}

uint32_t MotorModel::param_raw(uint8_t rid) const
{
  const auto it = ram_.find(rid);
  return it == ram_.end() ? 0u : it->second;
}

void MotorModel::clear_setpoints()
{
  status_.last_mit = MitCommand{};
  posvel_ = PosVelCommand{};
  vel_ = 0.0;
  posforce_ = PosForceCommand{};
}

void MotorModel::trip(uint8_t code)
{
  if (status_.code >= kFaultThreshold) {return;}  // the first fault stays
  status_.code = code;
  status_.enabled = false;
  clear_setpoints();
}

void MotorModel::force_code(uint8_t code)
{
  forced_code_ = code >= kFaultThreshold ? (code & 0xF) : 0;
  if (forced_code_ != 0 && status_.powered) {trip(forced_code_);}
}

void MotorModel::set_temperature_target(double mos_c, double rotor_c, double rate_c_per_s)
{
  target_mos_ = mos_c;
  target_rotor_ = rotor_c;
  temperature_rate_ = rate_c_per_s;
}

void MotorModel::set_power(bool on, double now)
{
  if (on == status_.powered) {return;}
  status_.powered = on;
  status_.enabled = on ? options_.power_up_enabled : false;
  status_.code = 0;
  status_.first_mit_valid = false;
  status_.tau = 0.0;
  clear_setpoints();
  ram_ = flash_;  // registers written without a save are lost
  status_.mode = static_cast<ControlMode>(ram_[kRidControlMode]);
  last_control_time_ = now;
}

Reply MotorModel::on_control_frame(uint32_t offset, const Payload & data, double now)
{
  if (!status_.powered) {return {};}
  const Reply state{replies() ? ReplyKind::State : ReplyKind::None, {}};
  const uint8_t special = offset == 0 ? special_command(data) : 0;
  if (special == kCmdEnable) {
    status_.enable_frames++;
    if (status_.code >= kFaultThreshold && options_.enable_clears_fault && forced_code_ == 0) {status_.code = 0;}
    if (status_.code < kFaultThreshold) {
      status_.enabled = true;
      status_.first_mit_valid = false;
      clear_setpoints();
      last_control_time_ = now;
    }
    return state;
  }
  if (special == kCmdDisable) {
    status_.disable_frames++;
    status_.enabled = false;
    clear_setpoints();
    return state;
  }
  if (special == kCmdClearError) {
    if (forced_code_ == 0) {status_.code = 0;}
    return state;
  }
  if (special == kCmdSetZero) {
    zero_offset_ = joint_q_;
    return options_.reply_to_set_zero ? state : Reply{};
  }

  status_.control_frames++;
  if (status_.enabled) {
    last_control_time_ = now;
    if (offset == 0 && status_.mode == ControlMode::MIT) {
      status_.last_mit = decode_mit(data, limits_);
      if (!status_.first_mit_valid) {
        status_.first_mit_valid = true;
        status_.first_mit = status_.last_mit;
        status_.first_mit_position = status_.q;
      }
    } else if (offset == kPosVelOffset && status_.mode == ControlMode::POS_VEL) {
      posvel_ = decode_posvel(data);
    } else if (offset == kVelOffset && status_.mode == ControlMode::VEL) {
      vel_ = decode_vel(data);
    } else if (offset == kPosForceOffset && status_.mode == ControlMode::POS_FORCE) {
      posforce_ = decode_posforce(data);
    }
  }
  if (!status_.enabled && status_.code < kFaultThreshold && !options_.answers_when_disabled) {return {};}
  return state;
}

Reply MotorModel::on_config_frame(const ConfigFrame & frame, double /*now*/)
{
  if (!status_.powered) {return {};}
  if (frame.command == kCfgRefresh) {
    status_.refresh_frames++;
    if (!replies()) {return {};}
    if (!status_.enabled && status_.code < kFaultThreshold && !options_.answers_when_disabled) {return {};}
    return {ReplyKind::State, {}};
  }
  status_.param_frames++;
  ConfigFrame out = frame;
  if (frame.command == kCfgRead) {
    out.raw = param_raw(frame.rid);
  } else if (frame.command == kCfgWrite) {
    ram_[frame.rid] = frame.raw;
    if (frame.rid == kRidControlMode && frame.raw >= 1 && frame.raw <= 4) {
      status_.mode = static_cast<ControlMode>(frame.raw);
      clear_setpoints();
    }
  } else if (frame.command == kCfgSave) {
    if (status_.enabled) {return {};}  // a save is refused while enabled (openarm_can damiao_simulator.py)
    flash_ = ram_;
    out.rid = 0x01;
    out.raw = 0;
  } else {
    return {};
  }
  if (!replies()) {return {};}
  return {ReplyKind::Param, encode_config(out)};
}

std::pair<double, double> MotorModel::law(double q) const
{
  const double speed_gain =
    options_.vel_kd_scale * config_.peak_torque / (0.1 * std::max(config_.no_load_speed_24v, 1e-6));
  switch (status_.mode) {
    case ControlMode::MIT: {
      const MitCommand & c = status_.last_mit;
      return {c.kp * (c.q - q) + c.kd * c.dq + c.tau, c.kd};
    }
    case ControlMode::POS_VEL: {
      const double limit = std::fabs(posvel_.dq);
      return {speed_gain * clamp(options_.posvel_kp * (posvel_.q - q), -limit, limit), speed_gain};
    }
    case ControlMode::VEL:
      return {speed_gain * vel_, speed_gain};
    case ControlMode::POS_FORCE: {
      const double limit = std::fabs(posforce_.dq);
      return {speed_gain * clamp(options_.posvel_kp * (posforce_.q - q), -limit, limit), speed_gain};
    }
  }
  return {0.0, 0.0};
}

double MotorModel::step(double joint_q, double joint_dq, double now, double dt, double supply_v)
{
  joint_q_ = joint_q;
  status_.q = joint_q - zero_offset_;
  status_.dq = joint_dq;
  status_.t_mos = approach(status_.t_mos, target_mos_, temperature_rate_, dt);
  status_.t_rotor = approach(status_.t_rotor, target_rotor_, temperature_rate_, dt);

  if (!status_.powered) {
    status_.tau = 0.0;
    return 0.0;
  }
  if (forced_code_ != 0) {
    trip(forced_code_);
  } else if (supply_v > options_.over_voltage_v) {
    trip(kCodeOverVoltage);
  } else if (supply_v < options_.under_voltage_v) {
    trip(kCodeUnderVoltage);
  } else if (status_.t_mos > options_.over_temperature_c) {
    trip(kCodeMosOverTemperature);
  } else if (status_.t_rotor > options_.over_temperature_c) {
    trip(kCodeCoilOverTemperature);
  } else if (status_.enabled && options_.can_timeout_s > 0.0 && now - last_control_time_ > options_.can_timeout_s) {
    trip(kCodeCommunicationLost);
  }
  if (!status_.enabled) {
    status_.tau = 0.0;
    tau_slow_ = 0.0;
    return 0.0;
  }

  const double cap = std::min(config_.peak_torque, limits_.t_max);
  const std::pair<double, double> stiff_damping = law(status_.q);
  double tau = stiff_damping.first - stiff_damping.second * status_.dq;
  if (options_.implicit_damping && dt > 0.0) {
    // Implicit Euler on the damping term for the fast part of the torque only. tau_slow_ is the applied torque
    // low-passed at damping_split_hz; what the law asks beyond it is divided by 1 + damping * dt / inertia, which is
    // what an implicit step of the damping term on that inertia gives. Below the split frequency tau_slow_ follows
    // the law and nothing changes.
    const double inertia = std::max(options_.damping_inertia_safety * config_.damping_inertia, 1e-9);
    tau = tau_slow_ + (tau - tau_slow_) / (1.0 + stiff_damping.second * dt / inertia);
  }
  tau = clamp(tau, -cap, cap);
  if (options_.voltage_speed_model && tau * status_.dq > 0.0) {
    // Driving, not braking: the torque left falls linearly to zero at the no-load speed for this supply.
    const double no_load = config_.no_load_speed_24v * supply_v / 24.0;
    const double left = cap * std::max(0.0, 1.0 - std::fabs(status_.dq) / std::max(no_load, 1e-6));
    tau = clamp(tau, -left, left);
  }
  status_.tau = tau;
  const double split = 2.0 * M_PI * std::max(options_.damping_split_hz, 1e-3) * dt;
  tau_slow_ += split / (1.0 + split) * (tau - tau_slow_);
  return tau;
}

Payload MotorModel::state_frame()
{
  StateFeedback s;
  s.motor_id = static_cast<uint8_t>(config_.send_id & 0xF);
  s.code = status_.code >= kFaultThreshold ? status_.code : (status_.enabled ? kCodeEnabled : kCodeDisabled);
  s.q = status_.q;
  s.dq = status_.dq;
  s.tau = status_.tau;
  s.t_mos = static_cast<uint8_t>(clamp(std::round(status_.t_mos), 0.0, 255.0));
  s.t_rotor = static_cast<uint8_t>(clamp(std::round(status_.t_rotor), 0.0, 255.0));
  status_.replies++;
  return encode_state(s, limits_);
}

}  // namespace openarm_emulator
