#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

enum class DegradedMode
{
  VirtualWall,
  Unknown,
  MaxPlusOne,
};

struct WatchdogConfig
{
  int cloud_warn_timeout_ms{400};
  int cloud_fail_timeout_ms{800};
  int recovery_healthy_frames{3};
  DegradedMode degraded_mode{DegradedMode::VirtualWall};
  uint16_t virtual_wall_cm{80};
  uint16_t max_distance_cm{1000};
  double min_processing_rate_hz{5.0};
  int tf_failure_limit{3};
  int raw_cloud_empty_limit{3};
  int empty_cloud_limit{3};
  bool filtered_empty_is_clear{false};
  bool filtered_empty_warn_only{false};
};

struct WatchdogResult
{
  HealthStatus status{HealthStatus::OK};
  bool degraded{false};
  double cloud_age_ms{0.0};
  uint64_t transition_count{0};
  HealthStatus previous_status{HealthStatus::DEGRADED_CLOUD_TIMEOUT};
  int consecutive_tf_failures{0};
  int recovery_healthy_frames{0};
  std::string transition_reason{"STARTUP"};
  std::string text{"OK"};
};

class Watchdog
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit Watchdog(WatchdogConfig config = {});

  void markCloud(TimePoint now);
  void markRawCloudSize(std::size_t point_count);
  void markTfSuccess();
  void markTfFailure();
  void markFilteredCloudSize(std::size_t point_count);
  void markSourceInvalid(HealthStatus status);
  void markSourceValid();
  void setProcessingRateHz(double rate_hz);
  WatchdogResult evaluate(TimePoint now);
  SectorArray degradedDistances() const;
  void reset();
  const WatchdogConfig & config() const;

  static DegradedMode parseDegradedMode(const std::string & mode);

private:
  WatchdogConfig config_;
  TimePoint last_cloud_time_{};
  bool has_cloud_{false};
  uint64_t cloud_sequence_{0};
  uint64_t recovery_last_cloud_sequence_{0};
  int consecutive_tf_failures_{0};
  int consecutive_raw_empty_clouds_{0};
  int consecutive_empty_clouds_{0};
  double processing_rate_hz_{0.0};
  HealthStatus source_status_{HealthStatus::OK};
  bool fail_closed_latched_{true};
  int recovery_healthy_frames_{0};
  HealthStatus last_status_{HealthStatus::DEGRADED_CLOUD_TIMEOUT};
  HealthStatus previous_status_{HealthStatus::DEGRADED_CLOUD_TIMEOUT};
  uint64_t transition_count_{0};
  std::string transition_reason_{"STARTUP_NO_CLOUD"};

  uint16_t noObstacleCm() const;
};

}  // namespace aero_halo_360
