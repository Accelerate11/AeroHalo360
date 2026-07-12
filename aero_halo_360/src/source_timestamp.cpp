#include "aero_halo_360/source_timestamp.hpp"

#include <utility>

namespace aero_halo_360
{

SourceTimestampGuard::SourceTimestampGuard(SourceTimestampConfig config)
: config_(std::move(config))
{
}

SourceTimestampResult SourceTimestampGuard::evaluate(
  const int64_t stamp_ns,
  const int64_t now_ns)
{
  SourceTimestampResult result;
  result.sequence = sequence_;

  if (stamp_ns <= 0) {
    result.status = HealthStatus::DEGRADED_SOURCE_STAMP_ZERO;
    return result;
  }

  const bool relaxed = config_.mode == "replay" || config_.mode == "sim";
  result.age_ms = relaxed ?
    0.0 : static_cast<double>(now_ns - stamp_ns) / 1000000.0;

  if (!relaxed) {
    if (result.age_ms > static_cast<double>(config_.max_sensor_age_ms)) {
      result.status = HealthStatus::DEGRADED_SOURCE_STAMP_OLD;
      return result;
    }
    if (result.age_ms < -static_cast<double>(config_.future_tolerance_ms)) {
      result.status = HealthStatus::DEGRADED_SOURCE_STAMP_FUTURE;
      return result;
    }
    if (has_stamp_ && stamp_ns < last_stamp_ns_) {
      result.status = HealthStatus::DEGRADED_SOURCE_STAMP_BACKWARD;
      return result;
    }
    if (has_stamp_ && stamp_ns == last_stamp_ns_) {
      consecutive_repeated_ += 1;
      if (consecutive_repeated_ >= config_.max_repeated_frames) {
        result.status = HealthStatus::DEGRADED_SOURCE_STAMP_REPEATED;
      }
      return result;
    }
  }

  has_stamp_ = true;
  last_stamp_ns_ = stamp_ns;
  consecutive_repeated_ = 0;
  sequence_ += 1U;
  result.accepted = true;
  result.status = HealthStatus::OK;
  result.sequence = sequence_;
  return result;
}

void SourceTimestampGuard::reset()
{
  has_stamp_ = false;
  last_stamp_ns_ = 0;
  consecutive_repeated_ = 0;
  sequence_ = 0;
}

const SourceTimestampConfig & SourceTimestampGuard::config() const
{
  return config_;
}

}  // namespace aero_halo_360
