#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

struct DiagnosticsMetrics
{
  std::string input_topic;
  std::string input_frame;
  std::string target_frame;
  std::string input_qos{"SensorDataQoS/best_effort"};
  std::size_t input_publisher_count{0};
  double configured_publish_rate_hz{0.0};
  std::string profile_name{"unknown"};
  std::string safety_summary;
  std::string last_tf_error{"NONE"};
  int consecutive_tf_failures{0};
  std::string previous_status{"DEGRADED_CLOUD_TIMEOUT"};
  double cloud_interval_recent_ms{0.0};
  double cloud_interval_max_ms{0.0};
  double cloud_interval_p95_ms{0.0};
  double processing_latency_ms{0.0};
  double source_to_publish_ms{0.0};
  uint64_t rejected_pointcloud_count{0};
  std::string last_pointcloud_reject_reason{"NONE"};
  uint64_t transition_count{0};
  int recovery_healthy_frames{0};
  std::string transition_reason{"STARTUP"};
};

diagnostic_msgs::msg::DiagnosticArray makeDiagnostics(
  const std_msgs::msg::Header & header,
  HealthStatus status,
  const std::string & status_text,
  double cloud_age_ms,
  double processing_rate_hz,
  std::size_t filtered_points,
  bool degraded,
  const DiagnosticsMetrics & metrics = {});

}  // namespace aero_halo_360
