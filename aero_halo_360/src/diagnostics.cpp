#include "aero_halo_360/diagnostics.hpp"

#include <sstream>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace aero_halo_360
{
namespace
{
diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string numberToString(const double value)
{
  std::ostringstream stream;
  stream.precision(3);
  stream << std::fixed << value;
  return stream.str();
}
}  // namespace

diagnostic_msgs::msg::DiagnosticArray makeDiagnostics(
  const std_msgs::msg::Header & header,
  const HealthStatus status,
  const std::string & status_text,
  const double cloud_age_ms,
  const double processing_rate_hz,
  const std::size_t filtered_points,
  const bool degraded,
  const DiagnosticsMetrics & metrics)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header = header;

  diagnostic_msgs::msg::DiagnosticStatus item;
  item.name = "aero_halo_360/cloud_processor";
  item.hardware_id = "AeroHalo360";
  item.level = (degraded || status != HealthStatus::OK) ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  item.message = status_text;
  item.values.push_back(kv("status_code", std::to_string(static_cast<int>(status))));
  item.values.push_back(kv("cloud_age_ms", numberToString(cloud_age_ms)));
  item.values.push_back(kv("processing_rate_hz", numberToString(processing_rate_hz)));
  item.values.push_back(kv("filtered_points", std::to_string(filtered_points)));
  item.values.push_back(kv("degraded", degraded ? "true" : "false"));
  item.values.push_back(kv("input_topic", metrics.input_topic));
  item.values.push_back(kv("input_frame", metrics.input_frame));
  item.values.push_back(kv("target_frame", metrics.target_frame));
  item.values.push_back(kv("input_qos", metrics.input_qos));
  item.values.push_back(kv(
    "input_publisher_count", std::to_string(metrics.input_publisher_count)));
  item.values.push_back(kv(
    "configured_publish_rate_hz", numberToString(metrics.configured_publish_rate_hz)));
  item.values.push_back(kv("profile_name", metrics.profile_name));
  item.values.push_back(kv("safety_summary", metrics.safety_summary));
  item.values.push_back(kv("last_tf_error", metrics.last_tf_error));
  item.values.push_back(kv(
    "consecutive_tf_failures", std::to_string(metrics.consecutive_tf_failures)));
  item.values.push_back(kv("previous_status", metrics.previous_status));
  item.values.push_back(kv(
    "cloud_interval_recent_ms", numberToString(metrics.cloud_interval_recent_ms)));
  item.values.push_back(kv(
    "cloud_interval_max_ms", numberToString(metrics.cloud_interval_max_ms)));
  item.values.push_back(kv(
    "cloud_interval_p95_ms", numberToString(metrics.cloud_interval_p95_ms)));
  item.values.push_back(kv(
    "processing_latency_ms", numberToString(metrics.processing_latency_ms)));
  item.values.push_back(kv(
    "source_to_publish_ms", numberToString(metrics.source_to_publish_ms)));
  item.values.push_back(kv(
    "rejected_pointcloud_count", std::to_string(metrics.rejected_pointcloud_count)));
  item.values.push_back(kv(
    "last_pointcloud_reject_reason", metrics.last_pointcloud_reject_reason));
  item.values.push_back(kv(
    "transition_count", std::to_string(metrics.transition_count)));
  item.values.push_back(kv(
    "recovery_healthy_frames", std::to_string(metrics.recovery_healthy_frames)));
  item.values.push_back(kv("transition_reason", metrics.transition_reason));

  array.status.push_back(item);
  return array;
}

}  // namespace aero_halo_360
