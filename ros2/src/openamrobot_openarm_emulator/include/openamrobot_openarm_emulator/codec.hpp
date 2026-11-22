// SPDX-License-Identifier: MIT
// Damiao frame layout as the OpenArm host library uses it (enactic/openarm_can 1.4.1, dm_motor_control.cpp).
// Written independently of that library on purpose: test_codec_upstream compares the two bit for bit.
#pragma once

#include <array>
#include <cstdint>

namespace openarm_emulator
{

using Payload = std::array<uint8_t, 8>;

enum class MotorType { DM4310, DM4340, DM8009 };

/// Encoding ranges (openarm_can MOTOR_LIMIT_PARAMS). These scale the frames. They are not joint limits.
struct EncodingLimits
{
  double p_max;  // rad
  double v_max;  // rad/s
  double t_max;  // N m
};

EncodingLimits encoding_limits(MotorType type);
const char * motor_type_name(MotorType type);
bool motor_type_from_name(const char * name, MotorType & type);

constexpr double kKpMax = 500.0;
constexpr double kKdMax = 5.0;

constexpr uint32_t kConfigCanId = 0x7FF;
constexpr uint32_t kPosVelOffset = 0x100;
constexpr uint32_t kVelOffset = 0x200;
constexpr uint32_t kPosForceOffset = 0x300;

// Last byte of the FF FF FF FF FF FF FF xx frames.
constexpr uint8_t kCmdClearError = 0xFB;
constexpr uint8_t kCmdEnable = 0xFC;
constexpr uint8_t kCmdDisable = 0xFD;
constexpr uint8_t kCmdSetZero = 0xFE;

// Byte D2 of a 0x7FF frame.
constexpr uint8_t kCfgRead = 0x33;
constexpr uint8_t kCfgWrite = 0x55;
constexpr uint8_t kCfgSave = 0xAA;
constexpr uint8_t kCfgRefresh = 0xCC;

// Status and error codes, high nibble of D0 of a state frame (openarm_can MotorError).
constexpr uint8_t kCodeDisabled = 0x0;
constexpr uint8_t kCodeEnabled = 0x1;
constexpr uint8_t kCodeOverVoltage = 0x8;
constexpr uint8_t kCodeUnderVoltage = 0x9;
constexpr uint8_t kCodeOverCurrent = 0xA;
constexpr uint8_t kCodeMosOverTemperature = 0xB;
constexpr uint8_t kCodeCoilOverTemperature = 0xC;
constexpr uint8_t kCodeCommunicationLost = 0xD;
constexpr uint8_t kCodeOverload = 0xE;
constexpr uint8_t kFaultThreshold = 0x8;

// Register ids the emulator knows by name (openarm_can RID).
constexpr uint8_t kRidUnderVoltage = 0;
constexpr uint8_t kRidOverTemperature = 2;
constexpr uint8_t kRidMasterId = 7;
constexpr uint8_t kRidSlaveId = 8;
constexpr uint8_t kRidTimeout = 9;
constexpr uint8_t kRidControlMode = 10;
constexpr uint8_t kRidGearRatio = 20;
constexpr uint8_t kRidPMax = 21;
constexpr uint8_t kRidVMax = 22;
constexpr uint8_t kRidTMax = 23;
constexpr uint8_t kRidOverVoltage = 29;
constexpr uint8_t kRidCanBitrate = 35;

/// True for the registers the host reads as uint32 (7 to 10, 13 to 16, 35 to 36). The rest are float.
bool rid_is_uint32(uint8_t rid);

/// Clamp, scale to 2^bits - 1, truncate. Same arithmetic as CanPacketEncoder::double_to_uint.
uint16_t float_to_uint(double x, double x_min, double x_max, int bits);
double uint_to_float(uint16_t x, double x_min, double x_max, int bits);

struct MitCommand
{
  double q{0.0};
  double dq{0.0};
  double kp{0.0};
  double kd{0.0};
  double tau{0.0};
};

Payload encode_mit(const MitCommand & cmd, const EncodingLimits & limits);
MitCommand decode_mit(const Payload & data, const EncodingLimits & limits);

struct StateFeedback
{
  uint8_t motor_id{0};  // low nibble of D0
  uint8_t code{0};      // high nibble of D0
  double q{0.0};
  double dq{0.0};
  double tau{0.0};
  uint8_t t_mos{0};
  uint8_t t_rotor{0};
};

Payload encode_state(const StateFeedback & state, const EncodingLimits & limits);
StateFeedback decode_state(const Payload & data, const EncodingLimits & limits);

/// 0 when the payload is not FF FF FF FF FF FF FF xx, else xx.
uint8_t special_command(const Payload & data);
Payload encode_special(uint8_t command);

struct ConfigFrame
{
  uint16_t motor_id{0};  // D0 low, D1 high
  uint8_t command{0};    // D2
  uint8_t rid{0};        // D3
  uint32_t raw{0};       // D4..D7, little endian
};

ConfigFrame decode_config(const Payload & data);
Payload encode_config(const ConfigFrame & frame);

uint32_t float_bits(float value);
float bits_float(uint32_t raw);

struct PosVelCommand { double q{0.0}; double dq{0.0}; };
struct PosForceCommand { double q{0.0}; double dq{0.0}; double i_pu{0.0}; };

PosVelCommand decode_posvel(const Payload & data);
Payload encode_posvel(const PosVelCommand & cmd);
double decode_vel(const Payload & data);
PosForceCommand decode_posforce(const Payload & data);

}  // namespace openarm_emulator
