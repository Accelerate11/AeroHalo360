#include "aero_halo_360/watchdog.hpp"

#include <algorithm>
#include <stdexcept>

namespace aero_halo_360
{

Watchdog::Watchdog(WatchdogConfig config)
: config_(config)
{
}

void Watchdog::markCloud(const TimePoint now)
{
  last_cloud_time_ = now;
  has_cloud_ = true;
  cloud_sequence_ += 1;
}

void Watchdog::markRawCloudSize(const std::size_t point_count)
{
  consecutive_raw_empty_clouds_ = point_count == 0 ?
    consecutive_raw_empty_clouds_ + 1 : 0;
}

void Watchdog::markTfSuccess()
{
  consecutive_tf_failures_ = 0;
}

void Watchdog::markTfFailure()
{
  consecutive_tf_failures_ += 1;
}

void Watchdog::markFilteredCloudSize(const std::size_t point_count)
{
  consecutive_empty_clouds_ = point_count == 0 ?
    consecutive_empty_clouds_ + 1 : 0;
}

void Watchdog::markSourceInvalid(const HealthStatus status)
{
  source_status_ = status;
}

void Watchdog::markSourceValid()
{
  source_status_ = HealthStatus::OK;
}

void Watchdog::setProcessingRateHz(const double rate_hz)
{
  processing_rate_hz_ = rate_hz;
}

WatchdogResult Watchdog::evaluate(const TimePoint now)
{
  WatchdogResult result;
  if (!has_cloud_) {
    result.status = HealthStatus::DEGRADED_CLOUD_TIMEOUT;
    result.degraded = true;
    result.cloud_age_ms = static_cast<double>(config_.cloud_fail_timeout_ms);
  } else {
    result.cloud_age_ms = static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cloud_time_).count());
    result.cloud_age_ms = std::max(0.0, result.cloud_age_ms);
  }

  if (!has_cloud_ || result.cloud_age_ms > static_cast<double>(config_.cloud_fail_timeout_ms)) {
    result.status = HealthStatus::DEGRADED_CLOUD_TIMEOUT;
    result.degraded = true;
  } else if (source_status_ != HealthStatus::OK) {
    result.status = source_status_;
    result.degraded = true;
  } else if (consecutive_tf_failures_ >= config_.tf_failure_limit) {
    result.status = HealthStatus::DEGRADED_TF_FAILED;
    result.degraded = true;
  } else if (consecutive_raw_empty_clouds_ >= config_.raw_cloud_empty_limit) {
    result.status = HealthStatus::DEGRADED_RAW_CLOUD_EMPTY;
    result.degraded = true;
  } else if (consecutive_empty_clouds_ >= config_.empty_cloud_limit) {
    if (config_.filtered_empty_is_clear) {
      if (config_.filtered_empty_warn_only) {
        result.status = HealthStatus::WARN_FILTERED_CLOUD_EMPTY;
      }
    } else {
      result.status = HealthStatus::DEGRADED_EMPTY_CLOUD;
      result.degraded = true;
    }
  } else if (processing_rate_hz_ > 0.0 && processing_rate_hz_ < config_.min_processing_rate_hz) {
    result.status = HealthStatus::DEGRADED_LOW_RATE;
    result.degraded = true;
  } else if (result.cloud_age_ms > static_cast<double>(config_.cloud_warn_timeout_ms)) {
    result.status = HealthStatus::WARN_CLOUD_STALE;
  }

  const bool hard_failure = result.degraded;
  if (hard_failure) {
    fail_closed_latched_ = true;
    recovery_healthy_frames_ = 0;
  } else if (fail_closed_latched_) {
    if (result.status == HealthStatus::OK &&
      cloud_sequence_ != recovery_last_cloud_sequence_)
    {
      recovery_healthy_frames_ += 1;
      recovery_last_cloud_sequence_ = cloud_sequence_;
    } else if (result.status != HealthStatus::OK) {
      recovery_healthy_frames_ = 0;
    }
    if (recovery_healthy_frames_ < config_.recovery_healthy_frames) {
      result.status = HealthStatus::DEGRADED_CLOUD_TIMEOUT;
      result.degraded = true;
    } else {
      fail_closed_latched_ = false;
      recovery_healthy_frames_ = 0;
    }
  }

  if (result.status != last_status_) {
    transition_count_ += 1;
    previous_status_ = last_status_;
    transition_reason_ = healthStatusText(last_status_) + " -> " + healthStatusText(result.status);
    last_status_ = result.status;
  }

  result.text = healthStatusText(result.status);
  result.transition_count = transition_count_;
  result.previous_status = previous_status_;
  result.consecutive_tf_failures = consecutive_tf_failures_;
  result.recovery_healthy_frames = recovery_healthy_frames_;
  result.transition_reason = transition_reason_;
  return result;
}

SectorArray Watchdog::degradedDistances() const
{
  SectorArray output;
  switch (config_.degraded_mode) {
    case DegradedMode::VirtualWall: output.fill(config_.virtual_wall_cm); break;
    case DegradedMode::Unknown: output.fill(kUnknownDistanceCm); break;
    case DegradedMode::MaxPlusOne: output.fill(noObstacleCm()); break;
  }
  return output;
}

void Watchdog::reset()
{
  has_cloud_ = false;
  cloud_sequence_ = 0;
  recovery_last_cloud_sequence_ = 0;
  consecutive_tf_failures_ = 0;
  consecutive_raw_empty_clouds_ = 0;
  consecutive_empty_clouds_ = 0;
  processing_rate_hz_ = 0.0;
  source_status_ = HealthStatus::OK;
  fail_closed_latched_ = true;
  recovery_healthy_frames_ = 0;
  last_status_ = HealthStatus::DEGRADED_CLOUD_TIMEOUT;
  previous_status_ = HealthStatus::DEGRADED_CLOUD_TIMEOUT;
  transition_count_ = 0;
  transition_reason_ = "RESET_NO_CLOUD";
}

const WatchdogConfig & Watchdog::config() const
{
  return config_;
}

DegradedMode Watchdog::parseDegradedMode(const std::string & mode)
{
  if (mode == "virtual_wall") {
    return DegradedMode::VirtualWall;
  }
  if (mode == "unknown") {
    return DegradedMode::Unknown;
  }
  if (mode == "max_plus_one") {
    return DegradedMode::MaxPlusOne;
  }
  throw std::invalid_argument(
          "watchdog.degraded_mode 只能是 virtual_wall、unknown 或 max_plus_one，当前为: " +
          mode);
}

uint16_t Watchdog::noObstacleCm() const
{
  return static_cast<uint16_t>(config_.max_distance_cm + 1U);
}

}  // namespace aero_halo_360
