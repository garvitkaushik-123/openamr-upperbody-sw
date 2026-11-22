// SPDX-License-Identifier: MIT
// The real upstream driver against the emulator, over Linux virtual CAN.
//
// The driver is openarm_hardware/OpenArmHW from enactic/openarm_ros2 at the pin of config/sil_pins.yaml, loaded as the
// ros2_control plugin it is and not modified. Each test drives its lifecycle the way the resource manager does: init,
// configure, activate, read and write at 100 Hz, deactivate. The motors are the emulator on vcan_arm_l (and vcan_arm_r
// for the two-arm case). The joints are a small test plant, one inertia per joint, so that no simulator is needed here;
// the launch test runs the same emulator against Gazebo.
//
// Several cases record what the driver does today, including behaviour the audit lists as gaps. Those are named
// KnownIssue_ or Fault_ and assert the present behaviour. They are a record, not an approval. If upstream changes the
// behaviour, the test fails and the record is updated.
//
// Without the vcan interfaces or without the driver plugin, every case is BLOCKED (a gtest skip). It is never a pass.
#include <gtest/gtest.h>

#include <net/if.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "blocked.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "openamrobot_openarm_emulator/arm_emulator.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace openarm_emulator;  // NOLINT
using hardware_interface::return_type;
using CallbackReturn = hardware_interface::CallbackReturn;

namespace
{
constexpr char kLeft[] = "vcan_arm_l";
constexpr char kRight[] = "vcan_arm_r";
constexpr size_t kJoints = 8;
// Gains of openarm_description 14ff67b control_gains.yaml, which the combined model passes to the driver.
const double kKp[7] = {70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
const double kKd[7] = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};

bool interface_exists(const char * name) {return if_nametoindex(name) != 0;}

/// Link up or down. Root, or passwordless sudo, or a helper named in OPENARM_EMULATOR_TEST_LINK_HELPER.
bool set_link(const char * name, bool up)
{
  const std::string state = up ? "up" : "down";
  if (const char * helper = std::getenv("OPENARM_EMULATOR_TEST_LINK_HELPER")) {
    return std::system((std::string(helper) + " " + name + " " + state).c_str()) == 0;
  }
  const std::string cmd = std::string("ip link set ") + name + " " + state + " 2>/dev/null";
  return std::system(cmd.c_str()) == 0 || std::system(("sudo -n " + cmd).c_str()) == 0;
}

/// Eight joints, each a rigid inertia with viscous friction and a constant load, stepped at 1 kHz of wall time.
/// The load on joint 2 stands in for gravity: with the motor off, the joint falls to its stop.
class Plant
{
public:
  explicit Plant(ArmEmulator & arm)
  : arm_(arm), q_(kJoints, 0.0), dq_(kJoints, 0.0), tau_(kJoints, 0.0), load_(kJoints, 0.0)
  {
    load_[1] = -3.0;
  }
  ~Plant() {stop();}
  void set_position(size_t joint, double q)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q_[joint] = q;
    dq_[joint] = 0.0;
  }
  double position(size_t joint)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return q_[joint];
  }
  void start()
  {
    running_ = true;
    thread_ = std::thread([this] {
          const double dt = 0.001;
          auto next = std::chrono::steady_clock::now();
          while (running_) {
            {
              std::lock_guard<std::mutex> lock(mutex_);
              arm_.step(q_, dq_, dt, tau_);
              for (size_t i = 0; i < kJoints; ++i) {
                dq_[i] += dt * (tau_[i] + load_[i] - kFriction[i] * dq_[i]) / kInertia[i];
                q_[i] += dt * dq_[i];
                if (std::fabs(q_[i]) > kStop) {
                  q_[i] = std::copysign(kStop, q_[i]);
                  dq_[i] = 0.0;
                }
              }
            }
            next += std::chrono::microseconds(1000);
            std::this_thread::sleep_until(next);
          }
        });
  }
  void stop()
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
  }
  static constexpr double kStop = 1.5;

private:
  static constexpr double kInertia[kJoints] = {0.05, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.002};
  static constexpr double kFriction[kJoints] = {0.5, 0.5, 0.5, 0.5, 0.05, 0.05, 0.05, 0.01};
  ArmEmulator & arm_;
  std::mutex mutex_;
  std::vector<double> q_, dq_, tau_, load_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

std::string urdf(const std::string & can_interface, const std::string & prefix, bool can_fd)
{
  // The <ros2_control> block is what the upstream macro expands to for one arm (openarm_description 14ff67b,
  // openarm.bimanual.ros2_control.xacro), with the CAN interface set.
  std::ostringstream s;
  s << "<?xml version=\"1.0\"?><robot name=\"driver_test\"><link name=\"l0\"/>";
  std::vector<std::string> joints;
  for (int i = 1; i <= 7; ++i) {joints.push_back("openarm_" + prefix + "joint" + std::to_string(i));}
  joints.push_back("openarm_" + prefix + "finger_joint1");
  for (size_t i = 0; i < joints.size(); ++i) {
    s << "<link name=\"l" << i + 1 << "\"/><joint name=\"" << joints[i] << "\" type=\"revolute\"><parent link=\"l" << i <<
      "\"/><child link=\"l" << i + 1 << "\"/><axis xyz=\"0 0 1\"/><limit lower=\"-3.14\" upper=\"3.14\" effort=\"40\" velocity=\"16\"/></joint>";
  }
  s << "<ros2_control name=\"openarm_" << prefix << "hardware_interface\" type=\"system\"><hardware>"
    "<plugin>openarm_hardware/OpenArmHW</plugin><param name=\"arm_type\">v20</param>"
    "<param name=\"can_interface\">" << can_interface << "</param><param name=\"arm_prefix\">" << prefix << "</param>"
    "<param name=\"can_fd\">" << (can_fd ? "true" : "false") << "</param><param name=\"ee_type\">pinch_gripper</param>";
  for (int i = 0; i < 7; ++i) {
    s << "<param name=\"kp" << i + 1 << "\">" << kKp[i] << "</param><param name=\"kd" << i + 1 << "\">" << kKd[i] << "</param>";
  }
  s << "</hardware>";
  for (const std::string & joint : joints) {
    s << "<joint name=\"" << joint << "\"><command_interface name=\"position\"/><command_interface name=\"velocity\"/>"
      "<command_interface name=\"effort\"/><state_interface name=\"position\"/><state_interface name=\"velocity\"/>"
      "<state_interface name=\"effort\"/></joint>";
  }
  s << "</ros2_control></robot>";
  return s.str();
}

/// One arm: emulator, plant and the driver plugin.
struct Arm
{
  std::unique_ptr<ArmEmulator> emulator;
  std::unique_ptr<Plant> plant;
  std::shared_ptr<hardware_interface::SystemInterface> driver;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
  rclcpp::Clock clock{RCL_STEADY_TIME};
  bool active{false};

  double state(size_t joint, size_t field) {return states[3 * joint + field].get_optional<double>().value();}
  double position(size_t joint) {return state(joint, 0);}
  void command(size_t joint, double q) {(void)commands[3 * joint].set_value(q);}

  /// One controller manager cycle: read, write. Both results are returned to be asserted.
  std::pair<return_type, return_type> cycle()
  {
    const rclcpp::Duration period(0, 10000000);
    const return_type r = driver->read(clock.now(), period);
    const return_type w = driver->write(clock.now(), period);
    return {r, w};
  }
  /// `seconds` of 100 Hz cycles. False if any read() or write() did not return OK.
  bool run(double seconds)
  {
    bool ok = true;
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < static_cast<int>(seconds * 100); ++i) {
      const auto result = cycle();
      ok = ok && result.first == return_type::OK && result.second == return_type::OK;
      next += std::chrono::milliseconds(10);
      std::this_thread::sleep_until(next);
    }
    return ok;
  }
  /// Commands start at the measured position, as a controller does when it activates.
  void hold_measured()
  {
    cycle();
    for (size_t j = 0; j < kJoints; ++j) {command(j, position(j));}
  }
};

class RealDriver : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
      "hardware_interface", "hardware_interface::SystemInterface");
  }
  static void TearDownTestSuite()
  {
    loader_.reset();
    rclcpp::shutdown();
  }

  /// Empty when the test can run, else why it is blocked.
  static std::string blocked()
  {
    if (!interface_exists(kLeft) || !interface_exists(kRight)) {
      return std::string("no ") + kLeft + " or " + kRight + " interface (scripts/setup_vcan.sh creates them; it needs the vcan kernel module)";
    }
    if (!loader_->isClassAvailable("openarm_hardware/OpenArmHW")) {
      return "the openarm_hardware/OpenArmHW plugin is not installed (build openarm_ros2 and openarm_can at the pins)";
    }
    return "";
  }

  std::unique_ptr<Arm> make_arm(const char * can_interface, const std::string & prefix, const MotorOptions & options = MotorOptions())
  {
    auto arm = std::make_unique<Arm>();
    ArmEmulatorConfig config;
    config.can_interface = can_interface;
    config.motors = default_arm_motors(prefix);
    config.options = options;
    arm->emulator = std::make_unique<ArmEmulator>(config);
    std::string error;
    if (!arm->emulator->start(error)) {
      ADD_FAILURE() << error;
      return nullptr;
    }
    arm->plant = std::make_unique<Plant>(*arm->emulator);
    return arm;
  }

  /// Load the plugin and take it to `inactive` (init and configure).
  void load_driver(Arm & arm, const char * can_interface, const std::string & prefix)
  {
    const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(can_interface, prefix, arm.emulator->fd_capable()));
    ASSERT_EQ(infos.size(), 1u);
    arm.driver = loader_->createSharedInstance("openarm_hardware/OpenArmHW");
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = infos[0];
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    ASSERT_EQ(arm.driver->init(params), CallbackReturn::SUCCESS);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    arm.states = arm.driver->export_state_interfaces();
    arm.commands = arm.driver->export_command_interfaces();
#pragma GCC diagnostic pop
    ASSERT_EQ(arm.states.size(), 3 * kJoints);
    ASSERT_EQ(arm.commands.size(), 3 * kJoints);
    ASSERT_EQ(arm.driver->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  /// on_activate, timed. It blocks for the driver's own return-to-zero ramp.
  double activate(Arm & arm)
  {
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(arm.driver->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    arm.active = true;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  }
  void deactivate(Arm & arm)
  {
    if (arm.active) {
      EXPECT_EQ(arm.driver->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      arm.active = false;
    }
  }

  /// The usual start: one arm on vcan_arm_l, plant running, driver active, commands at the measured position.
  void start_left(const MotorOptions & options = MotorOptions())
  {
    left_ = make_arm(kLeft, "left_", options);
    ASSERT_NE(left_, nullptr);
    left_->plant->start();
    load_driver(*left_, kLeft, "left_");
    if (HasFatalFailure()) {return;}
    activate(*left_);
    left_->hold_measured();
    ASSERT_TRUE(left_->run(0.3));
  }

  void SetUp() override
  {
    const std::string why = blocked();
    if (!why.empty()) {BLOCKED(why);}
  }
  void TearDown() override
  {
    for (Arm * arm : {left_.get(), right_.get()}) {
      if (arm == nullptr) {continue;}
      if (arm->driver) {deactivate(*arm);}
      arm->states.clear();
      arm->commands.clear();
      arm->driver.reset();
      arm->plant->stop();
      arm->emulator->stop();
    }
    if (link_was_set_down_) {set_link(kLeft, true);}
  }

  static std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> loader_;
  std::unique_ptr<Arm> left_, right_;
  bool link_was_set_down_{false};
};
std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> RealDriver::loader_;

}  // namespace

TEST_F(RealDriver, EnablesReadsAndCommandsOneJoint)
{
  left_ = make_arm(kLeft, "left_");
  ASSERT_NE(left_, nullptr);
  left_->plant->start();
  load_driver(*left_, kLeft, "left_");
  ASSERT_FALSE(HasFatalFailure());
  RecordProperty("can_fd", left_->emulator->fd_capable() ? "true" : "false");

  // init wrote CTRL_MODE = MIT to the gripper, configure sent one refresh to each motor. Nothing is enabled yet.
  EXPECT_GE(left_->emulator->status(7).param_frames, 1u);
  for (size_t i = 0; i < kJoints; ++i) {
    EXPECT_GE(left_->emulator->status(i).refresh_frames, 1u) << "motor " << i;
    EXPECT_FALSE(left_->emulator->status(i).enabled) << "motor " << i;
    EXPECT_EQ(left_->emulator->status(i).enable_frames, 0u) << "motor " << i;
  }

  // Enable.
  const double took = activate(*left_);
  RecordProperty("on_activate_seconds", std::to_string(took));
  for (size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(left_->emulator->status(i).enable_frames, 1u) << "motor " << i;
    EXPECT_TRUE(left_->emulator->status(i).enabled) << "motor " << i;
  }
  const ArmEmulatorStats stats = left_->emulator->stats();
  EXPECT_EQ(stats.rx_unknown, 0u);
  EXPECT_EQ(stats.tx_errors, 0u);
  if (left_->emulator->fd_capable()) {
    EXPECT_GT(stats.rx_fd, 0u);       // the driver sends CAN FD frames
    EXPECT_EQ(stats.rx_classic, 0u);
  } else {
    EXPECT_EQ(stats.rx_fd, 0u);
  }

  // Read.
  left_->hold_measured();
  ASSERT_TRUE(left_->run(0.3));
  for (size_t j = 0; j < 7; ++j) {EXPECT_NEAR(left_->position(j), left_->plant->position(j), 0.01) << "joint " << j + 1;}

  // Command joint 1 to 0.4 rad through the position command interface. The driver wraps it in an MIT frame with its gains.
  left_->command(0, 0.4);
  ASSERT_TRUE(left_->run(1.5));
  EXPECT_NEAR(left_->plant->position(0), 0.4, 0.01);    // the emulated motor moved the joint
  EXPECT_NEAR(left_->position(0), 0.4, 0.01);           // and the driver reads it back
  EXPECT_NEAR(left_->state(0, 1), 0.0, 0.05);           // velocity
  EXPECT_NEAR(left_->state(0, 2), 0.0, 0.3);            // effort: no load on joint 1
  EXPECT_NEAR(left_->state(1, 2), 3.0, 0.3);            // joint 2 holds its 3 N m load, and the driver reports the torque
  const MitCommand last = left_->emulator->status(0).last_mit;
  EXPECT_NEAR(last.q, 0.4, 1e-3);
  EXPECT_NEAR(last.kp, kKp[0], 0.13);
  EXPECT_NEAR(last.kd, kKd[0], 0.002);
  EXPECT_NEAR(last.dq, 0.0, 0.02);
  EXPECT_NEAR(last.tau, 0.0, 0.03);
  for (size_t j = 2; j < 7; ++j) {EXPECT_NEAR(left_->position(j), 0.0, 0.01) << "joint " << j + 1;}

  // The gripper: commanded like a joint, but its velocity and effort states are hard-coded to 0 in the driver.
  left_->command(7, 0.5);
  ASSERT_TRUE(left_->run(1.0));
  EXPECT_NEAR(left_->plant->position(7), 0.5, 0.02);
  EXPECT_NEAR(left_->position(7), 0.5, 0.02);
  EXPECT_EQ(left_->state(7, 1), 0.0);
  EXPECT_EQ(left_->state(7, 2), 0.0);
  EXPECT_NEAR(left_->emulator->status(7).last_mit.kp, 5.0, 0.13);   // GRIPPER_KP, GRIPPER_KD: the xacro passes no kp_hand
  EXPECT_NEAR(left_->emulator->status(7).last_mit.kd, 0.1, 0.002);

  // Disable: three disable frames per motor.
  deactivate(*left_);
  for (size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(left_->emulator->status(i).disable_frames, 3u) << "motor " << i;
    EXPECT_FALSE(left_->emulator->status(i).enabled) << "motor " << i;
  }
}

TEST_F(RealDriver, KnownIssue_ActivateMovesEveryJointToZero)
{
  left_ = make_arm(kLeft, "left_");
  ASSERT_NE(left_, nullptr);
  left_->plant->set_position(0, 0.6);
  left_->plant->set_position(3, 0.8);
  left_->plant->set_position(5, -0.5);
  left_->plant->start();
  load_driver(*left_, kLeft, "left_");
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_NEAR(left_->plant->position(0), 0.6, 1e-6);  // inactive: nothing moves

  const double took = activate(*left_);
  RecordProperty("on_activate_seconds", std::to_string(took));

  // 1. The first MIT frame after enable is a step to zero at full gains, sent while the joint is somewhere else.
  const MotorStatus j1 = left_->emulator->status(0);
  ASSERT_TRUE(j1.first_mit_valid);
  EXPECT_NEAR(j1.first_mit.q, 0.0, 1e-3);
  EXPECT_NEAR(j1.first_mit.kp, kKp[0], 0.13);
  EXPECT_NEAR(j1.first_mit_position, 0.6, 0.01);
  const MotorStatus j4 = left_->emulator->status(3);
  EXPECT_NEAR(j4.first_mit.q, 0.0, 1e-3);
  EXPECT_NEAR(j4.first_mit_position, 0.8, 0.01);
  // 2. The gripper is sent 0.044 rad: the 1.0 gripper's open width in metres, unconverted.
  EXPECT_NEAR(left_->emulator->status(7).first_mit.q, 0.044, 1e-3);
  // 3. Activation blocks for the ramp: 201 steps of 10 ms, plus the 100 ms after enable.
  EXPECT_GT(took, 2.0);
  EXPECT_LT(took, 4.0);
  // 4. Afterwards every arm joint is at zero. Nothing planned the motion or checked it for collision.
  EXPECT_NEAR(left_->plant->position(0), 0.0, 0.02);
  EXPECT_NEAR(left_->plant->position(3), 0.0, 0.02);
  EXPECT_NEAR(left_->plant->position(5), 0.0, 0.02);
}

TEST_F(RealDriver, Fault_CanLoss_DriverReportsHealthyAndFrozenState)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  left_->command(0, 0.3);
  ASSERT_TRUE(left_->run(1.0));
  ASSERT_NEAR(left_->position(0), 0.3, 0.01);

  left_->emulator->set_can_loss(true);   // the wire to the motors is open. The interface stays up.
  ASSERT_TRUE(left_->run(0.1));          // frames already in the socket are read
  const double frozen = left_->position(0);
  left_->command(0, 0.9);
  // Known issue: read() and write() keep returning OK. The state is the last value, with no sign that it is stale.
  EXPECT_TRUE(left_->run(1.0));
  EXPECT_EQ(left_->position(0), frozen);
  EXPECT_NEAR(left_->plant->position(0), 0.3, 0.01);  // the motor holds its last setpoint: the new command never arrived
  EXPECT_TRUE(left_->emulator->status(0).enabled);

  left_->emulator->set_can_loss(false);
  EXPECT_TRUE(left_->run(1.5));
  EXPECT_NEAR(left_->position(0), 0.9, 0.02);         // no recovery step is needed: the stream just resumes
}

TEST_F(RealDriver, Fault_BusDown_WritesFailAndDriverReportsHealthy)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  if (!set_link(kLeft, false)) {
    BLOCKED("cannot set " << kLeft << " down (needs root or passwordless sudo)");
  }
  link_was_set_down_ = true;
  ASSERT_TRUE(left_->run(0.1));
  const double frozen = left_->position(0);
  left_->command(0, 0.7);
  // Every write fails with ENETDOWN inside openarm_can. It counts the failure, and OpenArmHW never reads the counter.
  EXPECT_TRUE(left_->run(0.5));
  EXPECT_EQ(left_->position(0), frozen);
  EXPECT_NEAR(left_->plant->position(0), 0.0, 0.02);
  ASSERT_TRUE(set_link(kLeft, true));
  EXPECT_TRUE(left_->run(1.5));
  EXPECT_NEAR(left_->position(0), 0.7, 0.02);  // vcan has no bus-off: once the link is up the stream resumes
}

TEST_F(RealDriver, Fault_SingleMotorTimeout_OnlyThatJointFreezes)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  left_->emulator->set_motor_silent(3, true);  // J4 stops answering. It still receives.
  ASSERT_TRUE(left_->run(0.1));
  const double frozen = left_->position(3);
  left_->command(3, 0.5);
  left_->command(0, 0.3);
  EXPECT_TRUE(left_->run(1.5));                        // read() and write() return OK
  EXPECT_EQ(left_->position(3), frozen);               // J4's state is stale
  EXPECT_NEAR(left_->plant->position(3), 0.5, 0.02);   // while the joint moved 0.5 rad
  EXPECT_NEAR(left_->position(0), 0.3, 0.01);          // the other joints are unaffected
}

TEST_F(RealDriver, Fault_OverVoltage_MotorsTripAndDriverReportsHealthy)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_NEAR(left_->plant->position(1), 0.0, 0.2);    // joint 2 holds its load, sagging: position gain only, no gravity term
  left_->emulator->set_supply_voltage(33.0);           // above the emulator's over-voltage threshold (an assumption: 32 V)
  // Every motor reports 0x8 in each state frame and stops applying torque. The driver decodes the code and never checks it.
  EXPECT_TRUE(left_->run(1.5));
  for (size_t i = 0; i < kJoints; ++i) {
    EXPECT_EQ(left_->emulator->status(i).code, kCodeOverVoltage) << "motor " << i;
    EXPECT_FALSE(left_->emulator->status(i).enabled) << "motor " << i;
  }
  EXPECT_NEAR(left_->plant->position(1), -Plant::kStop, 0.01);  // the loaded joint fell to its stop
  EXPECT_NEAR(left_->position(1), -Plant::kStop, 0.01);         // the driver shows the fall: the motors still answer
  EXPECT_EQ(left_->state(1, 2), left_->state(1, 2));            // effort is a number, about 0
  EXPECT_NEAR(left_->state(1, 2), 0.0, 0.1);
  // The supply returns to normal. The fault stays, and the driver has no frame that clears it.
  left_->emulator->set_supply_voltage(25.6);
  EXPECT_TRUE(left_->run(0.5));
  EXPECT_EQ(left_->emulator->status(1).code, kCodeOverVoltage);
}

TEST_F(RealDriver, Fault_OverTemperature_MotorTripsAndDriverReportsHealthy)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  left_->emulator->set_temperature_target(1, 130.0, 25.0, 200.0);  // J2 MOSFET to 130 C at 200 C/s
  EXPECT_TRUE(left_->run(1.5));
  EXPECT_EQ(left_->emulator->status(1).code, kCodeMosOverTemperature);
  EXPECT_NEAR(left_->plant->position(1), -Plant::kStop, 0.01);  // J2 let go
  EXPECT_NEAR(left_->position(1), -Plant::kStop, 0.01);
  EXPECT_TRUE(left_->emulator->status(0).enabled);              // the other motors run on
  // Temperature and error code are decoded by openarm_can and exported nowhere: the driver has position, velocity
  // and effort interfaces only.
  for (const auto & state : left_->states) {
    const std::string name = state.get_interface_name();
    EXPECT_TRUE(name == "position" || name == "velocity" || name == "effort") << name;
  }
}

TEST_F(RealDriver, Fault_EstopPowerCut_FrozenStateWhileTheArmFalls_NoReEnableWithoutReactivation)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  left_->command(0, 0.4);
  ASSERT_TRUE(left_->run(1.0));
  const uint64_t enables = left_->emulator->status(1).enable_frames;

  // E-stop: the arm supply is cut. The interface and the host stay up. The motors are silent.
  left_->emulator->set_power(false);
  ASSERT_TRUE(left_->run(0.1));
  const double frozen = left_->position(1);
  EXPECT_TRUE(left_->run(1.0));                                    // read() and write() return OK throughout
  EXPECT_EQ(left_->position(1), frozen);                           // the driver shows joint 2 where it was
  EXPECT_NEAR(left_->plant->position(1), -Plant::kStop, 0.01);     // the joint is at its stop, 1.5 rad away
  EXPECT_GT(std::fabs(left_->position(1) - left_->plant->position(1)), 1.0);

  // Reset: supply back. The motors answer again, in the power-up state (disabled: an assumption, README).
  left_->emulator->set_power(true);
  const uint64_t frames = left_->emulator->status(1).control_frames;
  EXPECT_TRUE(left_->run(1.0));
  EXPECT_NEAR(left_->position(1), -Plant::kStop, 0.01);            // the state is live again and shows the fall
  EXPECT_GT(left_->emulator->status(1).control_frames, frames);    // the driver keeps sending MIT setpoints
  EXPECT_FALSE(left_->emulator->status(1).enabled);                // which a disabled motor ignores
  EXPECT_EQ(left_->emulator->status(1).enable_frames, enables);    // and it never sends enable again by itself
  EXPECT_NEAR(left_->plant->position(1), -Plant::kStop, 0.01);

  // The only way back is to deactivate and activate the hardware component. That runs the move to zero again.
  deactivate(*left_);
  activate(*left_);
  EXPECT_EQ(left_->emulator->status(1).enable_frames, enables + 1);
  EXPECT_TRUE(left_->emulator->status(1).enabled);
  EXPECT_NEAR(left_->emulator->status(1).first_mit.q, 0.0, 1e-3);          // a step to zero ...
  EXPECT_NEAR(left_->emulator->status(1).first_mit_position, -Plant::kStop, 0.02);  // ... from the stop
  EXPECT_NEAR(left_->plant->position(1), 0.0, 0.2);
  EXPECT_NEAR(left_->plant->position(0), 0.0, 0.02);               // joint 1 was at 0.4 and went to zero too
}

TEST_F(RealDriver, SupplyVoltageAcrossTheRobotRangeRunsWithoutFault)
{
  start_left();
  ASSERT_FALSE(HasFatalFailure());
  double target = 0.2;
  for (double volts : {25.6, 27.4, 29.2}) {
    left_->emulator->set_supply_voltage(volts);
    left_->command(0, target);
    EXPECT_TRUE(left_->run(1.0)) << volts;
    EXPECT_NEAR(left_->position(0), target, 0.01) << volts;
    for (size_t i = 0; i < kJoints; ++i) {
      EXPECT_TRUE(left_->emulator->status(i).enabled) << volts << " V, motor " << i;
      EXPECT_EQ(left_->emulator->status(i).code, 0) << volts << " V, motor " << i;
    }
    target += 0.2;
  }
}

TEST_F(RealDriver, TwoArmsOnTwoInterfacesAreIndependent)
{
  left_ = make_arm(kLeft, "left_");
  right_ = make_arm(kRight, "right_");
  ASSERT_NE(left_, nullptr);
  ASSERT_NE(right_, nullptr);
  left_->plant->start();
  right_->plant->start();
  load_driver(*left_, kLeft, "left_");
  load_driver(*right_, kRight, "right_");
  ASSERT_FALSE(HasFatalFailure());
  activate(*left_);
  activate(*right_);
  left_->hold_measured();
  right_->hold_measured();
  left_->command(0, 0.3);
  right_->command(0, -0.3);
  for (int i = 0; i < 100; ++i) {
    left_->cycle();
    right_->cycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(left_->position(0), 0.3, 0.01);
  EXPECT_NEAR(right_->position(0), -0.3, 0.01);
  // The same ids on both buses, and no frame crosses over.
  EXPECT_EQ(left_->emulator->stats().rx_unknown, 0u);
  EXPECT_EQ(right_->emulator->stats().rx_unknown, 0u);

  // Power cut on the left arm only.
  left_->emulator->set_power(false);
  right_->command(0, -0.6);
  for (int i = 0; i < 100; ++i) {
    left_->cycle();
    right_->cycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(left_->plant->position(1), -Plant::kStop, 0.01);
  EXPECT_NEAR(right_->plant->position(1), 0.0, 0.2);
  EXPECT_NEAR(right_->position(0), -0.6, 0.01);
  for (size_t i = 0; i < kJoints; ++i) {EXPECT_TRUE(right_->emulator->status(i).enabled) << "motor " << i;}
}
