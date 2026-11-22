// SPDX-License-Identifier: MIT
// Frame encode and decode. First against fixed bytes, then bit for bit against the upstream library
// (enactic/openarm_can at the pin of config/sil_pins.yaml) when it is installed.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "blocked.hpp"
#include "openamrobot_openarm_emulator/codec.hpp"

#ifdef HAVE_OPENARM_CAN
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#endif

using namespace openarm_emulator;  // NOLINT

namespace
{
const MotorType kTypes[] = {MotorType::DM4310, MotorType::DM4340, MotorType::DM8009};
}

TEST(Codec, EncodingRangesAreTheUpstreamTable)
{
  EXPECT_EQ(encoding_limits(MotorType::DM4310).v_max, 30.0);
  EXPECT_EQ(encoding_limits(MotorType::DM4310).t_max, 10.0);
  EXPECT_EQ(encoding_limits(MotorType::DM4340).v_max, 10.0);
  EXPECT_EQ(encoding_limits(MotorType::DM4340).t_max, 28.0);
  EXPECT_EQ(encoding_limits(MotorType::DM8009).v_max, 45.0);
  EXPECT_EQ(encoding_limits(MotorType::DM8009).t_max, 54.0);
  for (MotorType t : kTypes) {EXPECT_EQ(encoding_limits(t).p_max, 12.5);}
}

TEST(Codec, MitZeroFrameIsTheKnownBytes)
{
  // All zero: q and dq and tau sit at mid range, truncated (0x7FFF, 0x7FF, 0x7FF), kp and kd at 0.
  const Payload zero = encode_mit(MitCommand{}, encoding_limits(MotorType::DM4310));
  EXPECT_EQ(zero, (Payload{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF}));
  // Full scale: q max, dq max, kp 500, kd 5, tau max.
  const EncodingLimits l = encoding_limits(MotorType::DM8009);
  EXPECT_EQ(encode_mit({l.p_max, l.v_max, 500.0, 5.0, l.t_max}, l), (Payload{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
  EXPECT_EQ(encode_mit({-l.p_max, -l.v_max, 0.0, 0.0, -l.t_max}, l), (Payload{0, 0, 0, 0, 0, 0, 0, 0}));
  // Out of range values clamp. They do not wrap.
  EXPECT_EQ(encode_mit({100.0, 1000.0, 9999.0, 99.0, 1000.0}, l), (Payload{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
}

TEST(Codec, MitFieldsSitInTheirBits)
{
  const EncodingLimits l = encoding_limits(MotorType::DM4340);
  // kp 70 of 500 on 12 bits: 573 = 0x23D. kd 2 of 5: 1638 = 0x666.
  const Payload p = encode_mit({0.0, 0.0, 70.0, 2.0, 0.0}, l);
  EXPECT_EQ(p[3] & 0x0F, 0x02);
  EXPECT_EQ(p[4], 0x3D);
  EXPECT_EQ(p[5], 0x66);
  EXPECT_EQ(p[6] >> 4, 0x06);
  const MitCommand back = decode_mit(p, l);
  EXPECT_NEAR(back.kp, 70.0, 500.0 / 4095);
  EXPECT_NEAR(back.kd, 2.0, 5.0 / 4095);
  EXPECT_NEAR(back.q, 0.0, 25.0 / 65535);
}

TEST(Codec, StateFrameLayout)
{
  // The reply upstream's own simulated motor sends after a disable (openarm_can test/cli/damiao_simulator.py):
  // id in the low nibble, code 0, position 0x8000, velocity 0x800, torque 0x800, 25 C and 25 C.
  const Payload wire{0x03, 0x80, 0x00, 0x80, 0x08, 0x00, 25, 25};
  const EncodingLimits l = encoding_limits(MotorType::DM4340);
  const StateFeedback s = decode_state(wire, l);
  EXPECT_EQ(s.motor_id, 3);
  EXPECT_EQ(s.code, kCodeDisabled);
  EXPECT_NEAR(s.q, 0.0, 25.0 / 65535);
  EXPECT_NEAR(s.dq, 0.0, 20.0 / 4095);
  EXPECT_NEAR(s.tau, 0.0, 56.0 / 4095);
  EXPECT_EQ(s.t_mos, 25);
  EXPECT_EQ(s.t_rotor, 25);

  StateFeedback out;
  out.motor_id = 0x8;
  out.code = kCodeMosOverTemperature;
  out.q = l.p_max;
  out.dq = -l.v_max;
  out.tau = l.t_max;
  out.t_mos = 131;
  out.t_rotor = 77;
  EXPECT_EQ(encode_state(out, l), (Payload{0xB8, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 131, 77}));
}

TEST(Codec, SpecialAndConfigFrames)
{
  EXPECT_EQ(special_command({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}), kCmdEnable);
  EXPECT_EQ(special_command({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}), kCmdDisable);
  EXPECT_EQ(special_command({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}), kCmdSetZero);
  EXPECT_EQ(special_command({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}), kCmdClearError);
  EXPECT_EQ(special_command({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}), 0);  // the full-scale MIT frame is not a command
  EXPECT_EQ(special_command({0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF}), 0);

  const ConfigFrame refresh = decode_config({0x05, 0x00, 0xCC, 0, 0, 0, 0, 0});
  EXPECT_EQ(refresh.motor_id, 5);
  EXPECT_EQ(refresh.command, kCfgRefresh);
  const ConfigFrame write = decode_config({0x08, 0x00, 0x55, 0x0A, 0x01, 0x00, 0x00, 0x00});
  EXPECT_EQ(write.motor_id, 8);
  EXPECT_EQ(write.command, kCfgWrite);
  EXPECT_EQ(write.rid, kRidControlMode);
  EXPECT_EQ(write.raw, 1u);
  EXPECT_EQ(encode_config(write), (Payload{0x08, 0x00, 0x55, 0x0A, 0x01, 0x00, 0x00, 0x00}));
  EXPECT_TRUE(rid_is_uint32(7) && rid_is_uint32(10) && rid_is_uint32(13) && rid_is_uint32(16) && rid_is_uint32(35) && rid_is_uint32(36));
  EXPECT_FALSE(rid_is_uint32(0) || rid_is_uint32(6) || rid_is_uint32(11) || rid_is_uint32(21) || rid_is_uint32(29));
}

TEST(Codec, PositionVelocityFrame)
{
  const Payload p = encode_posvel({1.5, -2.25});
  EXPECT_EQ(p, (Payload{0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x10, 0xC0}));  // two little-endian floats
  EXPECT_DOUBLE_EQ(decode_posvel(p).q, 1.5);
  EXPECT_DOUBLE_EQ(decode_posvel(p).dq, -2.25);
  const PosForceCommand f = decode_posforce({0x00, 0x00, 0xC0, 0x3F, 0xE8, 0x03, 0x88, 0x13});
  EXPECT_DOUBLE_EQ(f.q, 1.5);
  EXPECT_DOUBLE_EQ(f.dq, 10.0);
  EXPECT_DOUBLE_EQ(f.i_pu, 0.5);
}

#ifdef HAVE_OPENARM_CAN
namespace dm = openarm::damiao_motor;

namespace
{
dm::MotorType upstream_type(MotorType t)
{
  return t == MotorType::DM4310 ? dm::MotorType::DM4310 : (t == MotorType::DM4340 ? dm::MotorType::DM4340 : dm::MotorType::DM8009);
}
Payload payload_of(const std::vector<uint8_t> & v)
{
  Payload p{};
  for (size_t i = 0; i < 8 && i < v.size(); ++i) {p[i] = v[i];}
  return p;
}
}  // namespace

TEST(CodecUpstream, EncodingRangesMatch)
{
  for (MotorType t : kTypes) {
    const dm::LimitParam up = dm::MOTOR_LIMIT_PARAMS[static_cast<int>(upstream_type(t))];
    const EncodingLimits own = encoding_limits(t);
    EXPECT_EQ(own.p_max, up.pMax);
    EXPECT_EQ(own.v_max, up.vMax);
    EXPECT_EQ(own.t_max, up.tMax);
  }
}

TEST(CodecUpstream, HostFramesDecodeToWhatUpstreamEncoded)
{
  std::mt19937 rng(20261114);
  for (MotorType t : kTypes) {
    const EncodingLimits l = encoding_limits(t);
    dm::Motor motor(upstream_type(t), 0x03, 0x13);
    std::uniform_real_distribution<double> q(-13.0, 13.0), dq(-1.1 * l.v_max, 1.1 * l.v_max), kp(-1.0, 520.0), kd(-0.1, 5.5),
    tau(-1.1 * l.t_max, 1.1 * l.t_max);
    for (int i = 0; i < 20000; ++i) {
      const dm::MITParam param{kp(rng), kd(rng), q(rng), dq(rng), tau(rng)};
      const dm::CANPacket packet = dm::CanPacketEncoder::create_mit_control_command(motor, param);
      ASSERT_EQ(packet.send_can_id, 0x03u);
      const Payload wire = payload_of(packet.data);
      // The emulator's own encoder gives the same bytes ...
      ASSERT_EQ(encode_mit({param.q, param.dq, param.kp, param.kd, param.tau}, l), wire);
      // ... and its decoder returns the value the bytes carry: the clamped input, one quantum low at most.
      const MitCommand got = decode_mit(wire, l);
      const auto near = [](double value, double input, double lo, double hi, int bits) {
          const double clamped = std::max(lo, std::min(input, hi));
          const double quantum = (hi - lo) / ((1 << bits) - 1);
          return value <= clamped + 1e-9 && value >= clamped - quantum - 1e-9;
        };
      ASSERT_TRUE(near(got.q, param.q, -l.p_max, l.p_max, 16));
      ASSERT_TRUE(near(got.dq, param.dq, -l.v_max, l.v_max, 12));
      ASSERT_TRUE(near(got.kp, param.kp, 0.0, 500.0, 12));
      ASSERT_TRUE(near(got.kd, param.kd, 0.0, 5.0, 12));
      ASSERT_TRUE(near(got.tau, param.tau, -l.t_max, l.t_max, 12));
    }
  }
}

TEST(CodecUpstream, CommandAndConfigFramesMatch)
{
  dm::Motor motor(dm::MotorType::DM4310, 0x08, 0x18);
  EXPECT_EQ(special_command(payload_of(dm::CanPacketEncoder::create_enable_command(motor).data)), kCmdEnable);
  EXPECT_EQ(special_command(payload_of(dm::CanPacketEncoder::create_disable_command(motor).data)), kCmdDisable);
  EXPECT_EQ(special_command(payload_of(dm::CanPacketEncoder::create_set_zero_command(motor).data)), kCmdSetZero);

  const dm::CANPacket refresh = dm::CanPacketEncoder::create_refresh_command(motor);
  EXPECT_EQ(refresh.send_can_id, kConfigCanId);
  EXPECT_EQ(decode_config(payload_of(refresh.data)).motor_id, 0x08);
  EXPECT_EQ(decode_config(payload_of(refresh.data)).command, kCfgRefresh);

  const dm::CANPacket query = dm::CanPacketEncoder::create_query_param_command(motor, static_cast<int>(dm::RID::PMAX));
  EXPECT_EQ(query.send_can_id, kConfigCanId);
  const ConfigFrame q = decode_config(payload_of(query.data));
  EXPECT_EQ(q.motor_id, 0x08);
  EXPECT_EQ(q.command, kCfgRead);
  EXPECT_EQ(q.rid, kRidPMax);

  const dm::CANPacket mode = dm::CanPacketEncoder::create_set_control_mode_command(motor, dm::ControlMode::POS_VEL);
  const ConfigFrame m = decode_config(payload_of(mode.data));
  EXPECT_EQ(m.command, kCfgWrite);
  EXPECT_EQ(m.rid, kRidControlMode);
  EXPECT_EQ(m.raw, 2u);

  const dm::CANPacket posvel = dm::CanPacketEncoder::create_posvel_control_command(motor, dm::PosVelParam{1.25, 3.5});
  EXPECT_EQ(posvel.send_can_id, 0x08u + kPosVelOffset);
  EXPECT_DOUBLE_EQ(decode_posvel(payload_of(posvel.data)).q, 1.25);
  EXPECT_DOUBLE_EQ(decode_posvel(payload_of(posvel.data)).dq, 3.5);

  const dm::CANPacket vel = dm::CanPacketEncoder::create_vel_control_command(motor, dm::VelParam{-4.5});
  EXPECT_EQ(vel.send_can_id, 0x08u + kVelOffset);
  EXPECT_DOUBLE_EQ(decode_vel(payload_of(vel.data)), -4.5);

  const dm::CANPacket force = dm::CanPacketEncoder::create_posforce_control_command(motor, dm::PosForceParam{0.5, 12.0, 0.25});
  EXPECT_EQ(force.send_can_id, 0x08u + kPosForceOffset);
  EXPECT_DOUBLE_EQ(decode_posforce(payload_of(force.data)).q, 0.5);
  EXPECT_DOUBLE_EQ(decode_posforce(payload_of(force.data)).dq, 12.0);
  EXPECT_DOUBLE_EQ(decode_posforce(payload_of(force.data)).i_pu, 0.25);
}

TEST(CodecUpstream, UpstreamDecodesTheEmulatorsStateFrames)
{
  std::mt19937 rng(7);
  for (MotorType t : kTypes) {
    const EncodingLimits l = encoding_limits(t);
    dm::Motor motor(upstream_type(t), 0x05, 0x15);
    std::uniform_real_distribution<double> q(-13.0, 13.0), dq(-1.1 * l.v_max, 1.1 * l.v_max), tau(-1.1 * l.t_max, 1.1 * l.t_max);
    std::uniform_int_distribution<int> byte(0, 255), code(0, 15);
    for (int i = 0; i < 20000; ++i) {
      StateFeedback s;
      s.motor_id = 0x5;
      s.code = static_cast<uint8_t>(code(rng));
      s.q = q(rng);
      s.dq = dq(rng);
      s.tau = tau(rng);
      s.t_mos = static_cast<uint8_t>(byte(rng));
      s.t_rotor = static_cast<uint8_t>(byte(rng));
      const Payload wire = encode_state(s, l);
      const dm::StateResult up = dm::CanPacketDecoder::parse_motor_state_data(motor, std::vector<uint8_t>(wire.begin(), wire.end()));
      ASSERT_TRUE(up.valid);
      const StateFeedback own = decode_state(wire, l);
      ASSERT_EQ(up.error_code, s.code);
      ASSERT_EQ(up.t_mos, s.t_mos);
      ASSERT_EQ(up.t_rotor, s.t_rotor);
      ASSERT_DOUBLE_EQ(up.position, own.q);
      ASSERT_DOUBLE_EQ(up.velocity, own.dq);
      ASSERT_DOUBLE_EQ(up.torque, own.tau);
      ASSERT_NEAR(up.position, std::max(-l.p_max, std::min(s.q, l.p_max)), 25.0 / 65535 + 1e-9);
      ASSERT_NEAR(up.velocity, std::max(-l.v_max, std::min(s.dq, l.v_max)), 2 * l.v_max / 4095 + 1e-9);
      ASSERT_NEAR(up.torque, std::max(-l.t_max, std::min(s.tau, l.t_max)), 2 * l.t_max / 4095 + 1e-9);
    }
  }
}

TEST(CodecUpstream, UpstreamDecodesTheEmulatorsParameterReplies)
{
  ConfigFrame f;
  f.motor_id = 0x08;
  f.command = kCfgRead;
  f.rid = kRidPMax;
  f.raw = float_bits(12.5f);
  Payload wire = encode_config(f);
  dm::ParamResult r = dm::CanPacketDecoder::parse_motor_param_data(std::vector<uint8_t>(wire.begin(), wire.end()));
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.rid, kRidPMax);
  EXPECT_DOUBLE_EQ(r.value, 12.5);

  f.command = kCfgWrite;
  f.rid = kRidControlMode;
  f.raw = 1;
  wire = encode_config(f);
  r = dm::CanPacketDecoder::parse_motor_param_data(std::vector<uint8_t>(wire.begin(), wire.end()));
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.rid, kRidControlMode);
  EXPECT_DOUBLE_EQ(r.value, 1.0);
  for (int rid = 0; rid < 82; ++rid) {
    // The same registers are integers on both sides.
    f.rid = static_cast<uint8_t>(rid);
    f.raw = 0x00000007;
    wire = encode_config(f);
    r = dm::CanPacketDecoder::parse_motor_param_data(std::vector<uint8_t>(wire.begin(), wire.end()));
    EXPECT_EQ(r.value == 7.0, rid_is_uint32(f.rid)) << "rid " << rid;
  }
}
#else
TEST(CodecUpstream, ComparisonWithOpenArmCan)
{
  BLOCKED("openarm_can is not installed, so the upstream layout comparison cannot run");
}
#endif
