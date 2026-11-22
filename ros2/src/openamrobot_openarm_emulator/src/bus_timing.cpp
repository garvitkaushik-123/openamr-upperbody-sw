// SPDX-License-Identifier: MIT
#include "openamrobot_openarm_emulator/bus_timing.hpp"

#include <algorithm>

namespace openarm_emulator
{

double frame_time_s(const BusTimingConfig & config, bool fd, bool brs)
{
  constexpr double stuff = 1.1;
  if (!fd) {
    // SOF 1, id 11, RTR 1, IDE 1, r0 1, DLC 4, data 64, CRC 15: stuffed. Then CRC delimiter, ACK 2, EOF 7, IFS 3.
    return (98.0 * stuff + 13.0) / config.nominal_bitrate;
  }
  // Nominal rate: SOF 1, id 11, RRS 1, IDE 1, FDF 1, res 1, BRS 1 (stuffed), then CRC delimiter 1, ACK 2, EOF 7, IFS 3.
  const double nominal_bits = 17.0 * stuff + 13.0;
  // Data rate: ESI 1, DLC 4, data 64 (stuffed), stuff count 4, CRC 17, fixed stuff bits 6.
  const double data_bits = 69.0 * stuff + 27.0;
  return nominal_bits / config.nominal_bitrate + data_bits / (brs ? config.data_bitrate : config.nominal_bitrate);
}

void BusTimingModel::push(const Frame & frame) {pending_.push_back(frame);}

void BusTimingModel::clear()
{
  pending_.clear();
  busy_ = false;
}

double BusTimingModel::next_wake() const
{
  if (busy_) {return current_.end;}
  if (pending_.empty()) {return std::numeric_limits<double>::infinity();}
  double first = pending_.front().ready_at;
  for (const Frame & f : pending_) {first = std::min(first, f.ready_at);}
  return config_.enabled ? std::max(first, free_at_) : first;
}

std::vector<BusTimingModel::Frame> BusTimingModel::advance(double now)
{
  std::vector<Frame> done;
  if (!config_.enabled) {
    for (Frame & f : pending_) {
      f.end = now;
      done.push_back(f);
    }
    pending_.clear();
    return done;
  }
  for (;;) {
    if (busy_) {
      if (current_.end > now) {break;}
      done.push_back(current_);
      busy_ = false;
      // One completion per call: the caller may add the answer to this frame before the next arbitration.
      break;
    }
    if (pending_.empty()) {break;}
    double first = pending_.front().ready_at;
    for (const Frame & f : pending_) {first = std::min(first, f.ready_at);}
    const double start = std::max(first, free_at_);
    if (start > now) {break;}
    // Arbitration: of the frames ready when the bus is free, the lowest id goes first. Ties keep arrival order.
    size_t best = pending_.size();
    for (size_t i = 0; i < pending_.size(); ++i) {
      if (pending_[i].ready_at <= start && (best == pending_.size() || pending_[i].can_id < pending_[best].can_id)) {
        best = i;
      }
    }
    current_ = pending_[best];
    pending_.erase(pending_.begin() + static_cast<long>(best));
    current_.end = start + frame_time_s(config_, current_.fd, current_.brs);
    free_at_ = current_.end;
    busy_ = true;
  }
  return done;
}

}  // namespace openarm_emulator
