// SPDX-License-Identifier: MIT
// One arm: eight emulated motors on one SocketCAN interface. A thread answers the host. step() runs the motor
// control laws against the joint state the caller supplies (Gazebo through ros2_control, or a test plant).
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "openamrobot_openarm_emulator/bus_timing.hpp"
#include "openamrobot_openarm_emulator/motor_model.hpp"

namespace openarm_emulator
{

/// The OpenArm 2.0 arm as upstream OpenArmHW addresses it: J1..J7 and the gripper, ids 0x01..0x08 and 0x11..0x18.
std::vector<MotorConfig> default_arm_motors(const std::string & arm_prefix);

struct ArmEmulatorConfig
{
  std::string can_interface{"vcan_arm_l"};
  std::vector<MotorConfig> motors;
  MotorOptions options;
  BusTimingConfig timing;
  bool reply_brs{true};          // BRS flag on FD replies. Check with candump on the bench.
  double supply_voltage{25.6};   // V
};

struct ArmEmulatorStats
{
  uint64_t rx_frames{0};
  uint64_t tx_frames{0};
  uint64_t tx_errors{0};
  uint64_t rx_errors{0};
  uint64_t rx_unknown{0};
  uint64_t rx_fd{0};
  uint64_t rx_classic{0};
  int last_tx_errno{0};
};

class ArmEmulator
{
public:
  explicit ArmEmulator(const ArmEmulatorConfig & config);
  ~ArmEmulator();

  /// Open the socket and start answering. False with a reason when the interface cannot be opened.
  bool start(std::string & error);
  void stop();
  bool running() const {return running_.load();}
  /// True when the interface carries CAN FD frames (MTU 72).
  bool fd_capable() const {return fd_capable_;}

  size_t size() const {return motors_.size();}
  /// One physics step for every motor. q, dq in, torque out, all in motor order. dt in seconds.
  void step(const std::vector<double> & q, const std::vector<double> & dq, double dt, std::vector<double> & tau);

  // Faults. Thread safe.
  void set_can_loss(bool lost);                      // the wire between adapter and motors is open
  void set_power(bool on);                           // false: E-stop power cut. true: supply back, power-up state
  void set_supply_voltage(double volts);
  void set_motor_silent(size_t index, bool silent);  // single-motor timeout: it runs but does not answer
  void force_code(size_t index, uint8_t code);
  void set_temperature_target(size_t index, double mos_c, double rotor_c, double rate_c_per_s);
  void set_timing(const BusTimingConfig & timing);

  bool can_loss() const {return can_loss_.load();}
  bool powered() const {return powered_.load();}
  double supply_voltage() const;
  MotorStatus status(size_t index) const;
  const MotorConfig & motor_config(size_t index) const {return configs_[index];}
  ArmEmulatorStats stats() const;

  /// Feed one frame as if the host had sent it. For tests without a CAN interface. Returns the replies, in bus order.
  struct WireFrame { uint32_t can_id; Payload data; bool fd; double time; };
  /// Replies whose modelled transmission ends by `until` are returned with that end time.
  std::vector<WireFrame> inject(const WireFrame & frame, double now, double until);

  static double clock_now();

private:
  struct PendingReply { size_t motor; Reply reply; };
  void run();
  void on_frame(uint32_t can_id, const Payload & data, bool fd, double now);
  void flush(double now, std::vector<WireFrame> * out);
  void send(const WireFrame & frame);

  ArmEmulatorConfig config_;
  std::vector<MotorConfig> configs_;
  mutable std::mutex mutex_;
  std::vector<MotorModel> motors_;
  BusTimingModel bus_;
  std::vector<PendingReply> replies_;   // indexed by token
  std::vector<uint64_t> free_tokens_;
  double supply_voltage_;
  ArmEmulatorStats stats_;
  std::atomic<bool> can_loss_{false};
  std::atomic<bool> powered_{true};
  std::atomic<bool> running_{false};
  bool fd_capable_{false};
  int fd_{-1};
  int wake_fd_{-1};
  std::thread thread_;
};

}  // namespace openarm_emulator
