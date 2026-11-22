// SPDX-License-Identifier: MIT
#include "openamrobot_openarm_emulator/arm_emulator.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>

namespace openarm_emulator
{

std::vector<MotorConfig> default_arm_motors(const std::string & arm_prefix)
{
  // Types and ids: openarm_ros2 4e837e1 openarm_simple_hardware.hpp. Peak torque and no-load speed at 24 V: the URDF
  // effort and velocity limits of openarm_description 14ff67b (40, 27, 7 N m; 160, 52, 200 rpm).
  const MotorType types[8] = {MotorType::DM8009, MotorType::DM8009, MotorType::DM4340, MotorType::DM4340,
    MotorType::DM4310, MotorType::DM4310, MotorType::DM4310, MotorType::DM4310};
  // kg m^2: each joint's child link, with what is fixed to it, about the joint axis. Computed from the expanded
  // combined model (test_package_static recomputes them). Both arms have the same values.
  const double damping_inertia[8] = {7.60e-4, 4.25e-4, 5.70e-4, 2.54e-3, 1.77e-4, 1.36e-4, 2.15e-3, 1.10e-4};
  std::vector<MotorConfig> motors;
  for (uint32_t i = 0; i < 8; ++i) {
    MotorConfig m;
    m.joint = "openarm_" + arm_prefix + (i < 7 ? "joint" + std::to_string(i + 1) : std::string("finger_joint1"));
    m.type = types[i];
    m.send_id = 0x01 + i;
    m.recv_id = 0x11 + i;
    m.peak_torque = types[i] == MotorType::DM8009 ? 40.0 : (types[i] == MotorType::DM4340 ? 27.0 : 7.0);
    m.no_load_speed_24v = types[i] == MotorType::DM8009 ? 16.755 : (types[i] == MotorType::DM4340 ? 5.4454 : 20.944);
    m.damping_inertia = damping_inertia[i];
    motors.push_back(m);
  }
  return motors;
}

double ArmEmulator::clock_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

ArmEmulator::ArmEmulator(const ArmEmulatorConfig & config)
: config_(config), configs_(config.motors), bus_(config.timing), supply_voltage_(config.supply_voltage)
{
  for (const MotorConfig & m : configs_) {motors_.emplace_back(m, config.options);}
}

ArmEmulator::~ArmEmulator() {stop();}

bool ArmEmulator::start(std::string & error)
{
  if (running_.load()) {return true;}
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    error = std::string("socket(PF_CAN): ") + std::strerror(errno);
    return false;
  }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, config_.can_interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    error = "no CAN interface " + config_.can_interface + ": " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  int on = 1;
  const bool fd_socket = ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) == 0;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    error = "bind " + config_.can_interface + ": " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  struct ifreq mtu;
  std::memset(&mtu, 0, sizeof(mtu));
  std::strncpy(mtu.ifr_name, config_.can_interface.c_str(), IFNAMSIZ - 1);
  fd_capable_ = fd_socket && ::ioctl(fd_, SIOCGIFMTU, &mtu) == 0 && mtu.ifr_mtu >= static_cast<int>(CANFD_MTU);
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  wake_fd_ = ::eventfd(0, EFD_NONBLOCK);
  running_.store(true);
  thread_ = std::thread(&ArmEmulator::run, this);
  return true;
}

void ArmEmulator::stop()
{
  if (!running_.exchange(false)) {return;}
  const uint64_t one = 1;
  if (wake_fd_ >= 0) {(void)!::write(wake_fd_, &one, sizeof(one));}
  if (thread_.joinable()) {thread_.join();}
  if (fd_ >= 0) {::close(fd_);}
  if (wake_fd_ >= 0) {::close(wake_fd_);}
  fd_ = wake_fd_ = -1;
}

void ArmEmulator::run()
{
  while (running_.load()) {
    double wake;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wake = bus_.next_wake();
    }
    const double wait = std::max(0.0, std::min(wake - clock_now(), 0.05));
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = static_cast<long>(wait * 1e9);
    struct pollfd fds[2] = {{fd_, POLLIN, 0}, {wake_fd_, POLLIN, 0}};
    ::ppoll(fds, 2, &ts, nullptr);
    if (!running_.load()) {break;}
    if (fds[0].revents != 0) {
      for (int n = 0; n < 256; ++n) {
        struct canfd_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        const ssize_t size = ::read(fd_, &frame, sizeof(frame));
        if (size < 0) {
          if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.rx_errors++;
            ::usleep(1000);  // link down: do not spin
          }
          break;
        }
        if (size != static_cast<ssize_t>(CAN_MTU) && size != static_cast<ssize_t>(CANFD_MTU)) {break;}
        if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) {continue;}
        Payload data{};
        std::memcpy(data.data(), frame.data, std::min<size_t>(frame.len, 8));
        std::lock_guard<std::mutex> lock(mutex_);
        on_frame(frame.can_id & CAN_SFF_MASK, data, size == static_cast<ssize_t>(CANFD_MTU), clock_now());
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    flush(clock_now(), nullptr);
  }
}

void ArmEmulator::on_frame(uint32_t can_id, const Payload & data, bool fd, double now)
{
  stats_.rx_frames++;
  (fd ? stats_.rx_fd : stats_.rx_classic)++;
  if (can_loss_.load()) {return;}  // the frame never reaches a motor
  size_t motor = motors_.size();
  Reply reply;
  if (can_id == kConfigCanId) {
    const ConfigFrame cfg = decode_config(data);
    for (size_t i = 0; i < motors_.size(); ++i) {
      if (configs_[i].send_id == cfg.motor_id) {
        motor = i;
        reply = motors_[i].on_config_frame(cfg, now);
        break;
      }
    }
  } else {
    for (size_t i = 0; i < motors_.size() && motor == motors_.size(); ++i) {
      for (uint32_t offset : {0u, kPosVelOffset, kVelOffset, kPosForceOffset}) {
        if (can_id == configs_[i].send_id + offset) {
          motor = i;
          reply = motors_[i].on_control_frame(offset, data, now);
          break;
        }
      }
    }
  }
  if (motor == motors_.size()) {
    stats_.rx_unknown++;
    return;
  }
  uint64_t token;
  if (free_tokens_.empty()) {
    token = replies_.size();
    replies_.push_back({motor, reply});
  } else {
    token = free_tokens_.back();
    free_tokens_.pop_back();
    replies_[token] = {motor, reply};
  }
  BusTimingModel::Frame frame;
  frame.ready_at = now;
  frame.can_id = can_id;
  frame.fd = fd;
  frame.brs = fd;  // the host sets BRS on every FD frame (openarm_can dm_motor_device.cpp)
  frame.reply = false;
  frame.token = token;
  bus_.push(frame);
}

void ArmEmulator::flush(double now, std::vector<WireFrame> * out)
{
  for (;;) {
    const std::vector<BusTimingModel::Frame> done = bus_.advance(now);
    if (done.empty()) {break;}
    for (const BusTimingModel::Frame & f : done) {
      PendingReply & pending = replies_[f.token];
      if (!f.reply) {
        if (pending.reply.kind == ReplyKind::None) {
          free_tokens_.push_back(f.token);
          continue;
        }
        BusTimingModel::Frame answer = f;
        answer.reply = true;
        answer.can_id = configs_[pending.motor].recv_id;
        answer.fd = f.fd && (fd_capable_ || fd_ < 0);
        answer.brs = answer.fd && config_.reply_brs;
        answer.ready_at = f.end + (bus_.config().enabled ? bus_.config().turnaround_s : 0.0);
        bus_.push(answer);
        continue;
      }
      MotorModel & motor = motors_[pending.motor];
      if (motor.replies() && !can_loss_.load()) {
        WireFrame wire;
        wire.can_id = f.can_id;
        wire.fd = f.fd;
        wire.time = f.end;
        wire.data = pending.reply.kind == ReplyKind::State ? motor.state_frame() : pending.reply.param;
        if (out != nullptr) {out->push_back(wire);} else {send(wire);}
      }
      free_tokens_.push_back(f.token);
    }
  }
}

void ArmEmulator::send(const WireFrame & wire)
{
  if (fd_ < 0) {return;}
  ssize_t written, expected;
  if (wire.fd) {
    struct canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = wire.can_id;
    frame.len = 8;
    frame.flags = config_.reply_brs ? CANFD_BRS : 0;
    std::memcpy(frame.data, wire.data.data(), 8);
    expected = sizeof(frame);
    written = ::write(fd_, &frame, sizeof(frame));
  } else {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = wire.can_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, wire.data.data(), 8);
    expected = sizeof(frame);
    written = ::write(fd_, &frame, sizeof(frame));
  }
  if (written == expected) {
    stats_.tx_frames++;
  } else {
    stats_.tx_errors++;
    stats_.last_tx_errno = errno;
  }
}

std::vector<ArmEmulator::WireFrame> ArmEmulator::inject(const WireFrame & frame, double now, double until)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<WireFrame> out;
  on_frame(frame.can_id, frame.data, frame.fd, now);
  flush(until, &out);
  return out;
}

void ArmEmulator::step(const std::vector<double> & q, const std::vector<double> & dq, double dt, std::vector<double> & tau)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = clock_now();
  tau.resize(motors_.size());
  for (size_t i = 0; i < motors_.size(); ++i) {tau[i] = motors_[i].step(q[i], dq[i], now, dt, supply_voltage_);}
}

void ArmEmulator::set_can_loss(bool lost)
{
  std::lock_guard<std::mutex> lock(mutex_);
  can_loss_.store(lost);
  if (lost) {
    bus_.clear();
    replies_.clear();
    free_tokens_.clear();
  }
}

void ArmEmulator::set_power(bool on)
{
  std::lock_guard<std::mutex> lock(mutex_);
  powered_.store(on);
  const double now = clock_now();
  for (MotorModel & m : motors_) {m.set_power(on, now);}
}

void ArmEmulator::set_supply_voltage(double volts)
{
  std::lock_guard<std::mutex> lock(mutex_);
  supply_voltage_ = volts;
}

double ArmEmulator::supply_voltage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return supply_voltage_;
}

void ArmEmulator::set_motor_silent(size_t index, bool silent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index < motors_.size()) {motors_[index].set_silent(silent);}
}

void ArmEmulator::force_code(size_t index, uint8_t code)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index < motors_.size()) {motors_[index].force_code(code);}
}

void ArmEmulator::set_temperature_target(size_t index, double mos_c, double rotor_c, double rate_c_per_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index < motors_.size()) {motors_[index].set_temperature_target(mos_c, rotor_c, rate_c_per_s);}
}

void ArmEmulator::set_timing(const BusTimingConfig & timing)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bus_.configure(timing);
}

MotorStatus ArmEmulator::status(size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return motors_.at(index).status();
}

ArmEmulatorStats ArmEmulator::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace openarm_emulator
