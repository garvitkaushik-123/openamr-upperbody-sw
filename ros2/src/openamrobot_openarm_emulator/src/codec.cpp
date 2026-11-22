// SPDX-License-Identifier: MIT
#include "openamrobot_openarm_emulator/codec.hpp"

#include <algorithm>
#include <cstring>

namespace openarm_emulator
{

EncodingLimits encoding_limits(MotorType type)
{
  switch (type) {
    case MotorType::DM4310: return {12.5, 30.0, 10.0};
    case MotorType::DM4340: return {12.5, 10.0, 28.0};
    case MotorType::DM8009: return {12.5, 45.0, 54.0};
  }
  return {12.5, 30.0, 10.0};
}

const char * motor_type_name(MotorType type)
{
  switch (type) {
    case MotorType::DM4310: return "DM4310";
    case MotorType::DM4340: return "DM4340";
    case MotorType::DM8009: return "DM8009";
  }
  return "unknown";
}

bool motor_type_from_name(const char * name, MotorType & type)
{
  for (MotorType t : {MotorType::DM4310, MotorType::DM4340, MotorType::DM8009}) {
    if (std::strcmp(name, motor_type_name(t)) == 0) {
      type = t;
      return true;
    }
  }
  return false;
}

bool rid_is_uint32(uint8_t rid)
{
  return (rid >= 7 && rid <= 10) || (rid >= 13 && rid <= 16) || (rid >= 35 && rid <= 36);
}

uint16_t float_to_uint(double x, double x_min, double x_max, int bits)
{
  x = std::max(x_min, std::min(x, x_max));
  const double norm = (x - x_min) / (x_max - x_min);
  return static_cast<uint16_t>(norm * ((1 << bits) - 1));
}

double uint_to_float(uint16_t x, double x_min, double x_max, int bits)
{
  return static_cast<double>(x) / ((1 << bits) - 1) * (x_max - x_min) + x_min;
}

Payload encode_mit(const MitCommand & cmd, const EncodingLimits & limits)
{
  const uint16_t q = float_to_uint(cmd.q, -limits.p_max, limits.p_max, 16);
  const uint16_t dq = float_to_uint(cmd.dq, -limits.v_max, limits.v_max, 12);
  const uint16_t kp = float_to_uint(cmd.kp, 0.0, kKpMax, 12);
  const uint16_t kd = float_to_uint(cmd.kd, 0.0, kKdMax, 12);
  const uint16_t tau = float_to_uint(cmd.tau, -limits.t_max, limits.t_max, 12);
  return {
    static_cast<uint8_t>(q >> 8), static_cast<uint8_t>(q & 0xFF),
    static_cast<uint8_t>(dq >> 4), static_cast<uint8_t>(((dq & 0xF) << 4) | ((kp >> 8) & 0xF)),
    static_cast<uint8_t>(kp & 0xFF), static_cast<uint8_t>(kd >> 4),
    static_cast<uint8_t>(((kd & 0xF) << 4) | ((tau >> 8) & 0xF)), static_cast<uint8_t>(tau & 0xFF)};
}

MitCommand decode_mit(const Payload & d, const EncodingLimits & limits)
{
  const uint16_t q = static_cast<uint16_t>((d[0] << 8) | d[1]);
  const uint16_t dq = static_cast<uint16_t>((d[2] << 4) | (d[3] >> 4));
  const uint16_t kp = static_cast<uint16_t>(((d[3] & 0xF) << 8) | d[4]);
  const uint16_t kd = static_cast<uint16_t>((d[5] << 4) | (d[6] >> 4));
  const uint16_t tau = static_cast<uint16_t>(((d[6] & 0xF) << 8) | d[7]);
  MitCommand cmd;
  cmd.q = uint_to_float(q, -limits.p_max, limits.p_max, 16);
  cmd.dq = uint_to_float(dq, -limits.v_max, limits.v_max, 12);
  cmd.kp = uint_to_float(kp, 0.0, kKpMax, 12);
  cmd.kd = uint_to_float(kd, 0.0, kKdMax, 12);
  cmd.tau = uint_to_float(tau, -limits.t_max, limits.t_max, 12);
  return cmd;
}

Payload encode_state(const StateFeedback & s, const EncodingLimits & limits)
{
  const uint16_t q = float_to_uint(s.q, -limits.p_max, limits.p_max, 16);
  const uint16_t dq = float_to_uint(s.dq, -limits.v_max, limits.v_max, 12);
  const uint16_t tau = float_to_uint(s.tau, -limits.t_max, limits.t_max, 12);
  return {
    static_cast<uint8_t>((s.motor_id & 0xF) | ((s.code & 0xF) << 4)),
    static_cast<uint8_t>(q >> 8), static_cast<uint8_t>(q & 0xFF),
    static_cast<uint8_t>(dq >> 4), static_cast<uint8_t>(((dq & 0xF) << 4) | ((tau >> 8) & 0xF)),
    static_cast<uint8_t>(tau & 0xFF), s.t_mos, s.t_rotor};
}

StateFeedback decode_state(const Payload & d, const EncodingLimits & limits)
{
  StateFeedback s;
  s.motor_id = d[0] & 0xF;
  s.code = d[0] >> 4;
  s.q = uint_to_float(static_cast<uint16_t>((d[1] << 8) | d[2]), -limits.p_max, limits.p_max, 16);
  s.dq = uint_to_float(static_cast<uint16_t>((d[3] << 4) | (d[4] >> 4)), -limits.v_max, limits.v_max, 12);
  s.tau = uint_to_float(static_cast<uint16_t>(((d[4] & 0xF) << 8) | d[5]), -limits.t_max, limits.t_max, 12);
  s.t_mos = d[6];
  s.t_rotor = d[7];
  return s;
}

uint8_t special_command(const Payload & d)
{
  for (int i = 0; i < 7; ++i) {
    if (d[i] != 0xFF) {return 0;}
  }
  return (d[7] >= kCmdClearError && d[7] <= kCmdSetZero) ? d[7] : 0;
}

Payload encode_special(uint8_t command)
{
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
}

ConfigFrame decode_config(const Payload & d)
{
  ConfigFrame f;
  f.motor_id = static_cast<uint16_t>(d[0] | (d[1] << 8));
  f.command = d[2];
  f.rid = d[3];
  std::memcpy(&f.raw, &d[4], 4);
  return f;
}

Payload encode_config(const ConfigFrame & f)
{
  Payload d{static_cast<uint8_t>(f.motor_id & 0xFF), static_cast<uint8_t>(f.motor_id >> 8), f.command, f.rid, 0, 0, 0, 0};
  std::memcpy(&d[4], &f.raw, 4);
  return d;
}

uint32_t float_bits(float value)
{
  uint32_t raw;
  std::memcpy(&raw, &value, 4);
  return raw;
}

float bits_float(uint32_t raw)
{
  float value;
  std::memcpy(&value, &raw, 4);
  return value;
}

PosVelCommand decode_posvel(const Payload & d)
{
  float q, dq;
  std::memcpy(&q, &d[0], 4);
  std::memcpy(&dq, &d[4], 4);
  return {q, dq};
}

Payload encode_posvel(const PosVelCommand & cmd)
{
  Payload d{};
  const float q = static_cast<float>(cmd.q), dq = static_cast<float>(cmd.dq);
  std::memcpy(&d[0], &q, 4);
  std::memcpy(&d[4], &dq, 4);
  return d;
}

double decode_vel(const Payload & d)
{
  float dq;
  std::memcpy(&dq, &d[0], 4);
  return dq;
}

PosForceCommand decode_posforce(const Payload & d)
{
  float q;
  std::memcpy(&q, &d[0], 4);
  const uint16_t v = static_cast<uint16_t>(d[4] | (d[5] << 8));
  const uint16_t i = static_cast<uint16_t>(d[6] | (d[7] << 8));
  return {q, v / 100.0, i / 10000.0};
}

}  // namespace openarm_emulator
