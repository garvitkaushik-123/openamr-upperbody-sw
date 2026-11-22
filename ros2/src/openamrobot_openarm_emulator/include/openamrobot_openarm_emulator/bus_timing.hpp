// SPDX-License-Identifier: MIT
// A model of one shared CAN bus in time. vcan delivers every frame at once and has no bit rate, so the emulator
// adds what a wire adds: each frame takes its length in bits, one frame at a time, lowest id first.
// Pure logic. Times are seconds on the caller's monotonic clock.
#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace openarm_emulator
{

struct BusTimingConfig
{
  bool enabled{true};
  double nominal_bitrate{1.0e6};   // bit/s, arbitration phase
  double data_bitrate{5.0e6};      // bit/s, data phase of an FD frame with BRS
  double turnaround_s{0.0};        // motor: end of the request to ready to answer. Not measured yet.
};

/// Wire time of one frame with 8 data bytes and an 11-bit id. An estimate: 10 % stuff bits assumed.
double frame_time_s(const BusTimingConfig & config, bool fd, bool brs);

class BusTimingModel
{
public:
  struct Frame
  {
    double ready_at{0.0};
    uint32_t can_id{0};
    bool fd{true};
    bool brs{true};
    bool reply{false};   // false: a host frame, already on vcan, modelled only for the time it takes
    uint64_t token{0};   // caller's handle
    double end{0.0};     // set when the frame leaves the model
  };

  explicit BusTimingModel(const BusTimingConfig & config = BusTimingConfig()) : config_(config) {}
  void configure(const BusTimingConfig & config) {config_ = config;}
  const BusTimingConfig & config() const {return config_;}

  void push(const Frame & frame);
  /// The next frame whose transmission has ended by `now`, at most one per call so that the caller can push the
  /// answer to it before the next arbitration. Call until it returns nothing. With the model disabled: every frame.
  std::vector<Frame> advance(double now);
  /// When to call advance() again. Infinity when nothing is pending.
  double next_wake() const;
  void clear();
  size_t pending() const {return pending_.size() + (busy_ ? 1 : 0);}

private:
  BusTimingConfig config_;
  std::vector<Frame> pending_;
  bool busy_{false};
  Frame current_{};
  double free_at_{-std::numeric_limits<double>::infinity()};
};

}  // namespace openarm_emulator
