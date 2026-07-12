#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>

namespace aero_halo_360
{

constexpr std::size_t kSectorCount = 72;
constexpr uint16_t kUnknownDistanceCm = std::numeric_limits<uint16_t>::max();

struct PointXYZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class HealthStatus : uint8_t
{
  OK = 0,
  DEGRADED_CLOUD_TIMEOUT = 1,
  DEGRADED_TF_FAILED = 2,
  DEGRADED_EMPTY_CLOUD = 3,
  DEGRADED_MAVLINK_ERROR = 4,
  DEGRADED_LOW_RATE = 5,
  DEGRADED_RAW_CLOUD_EMPTY = 6,
  WARN_FILTERED_CLOUD_EMPTY = 7,
  DEGRADED_SOURCE_STAMP_ZERO = 8,
  DEGRADED_SOURCE_STAMP_OLD = 9,
  DEGRADED_SOURCE_STAMP_REPEATED = 10,
  DEGRADED_SOURCE_STAMP_BACKWARD = 11,
  DEGRADED_SOURCE_STAMP_FUTURE = 12,
  DEGRADED_POINTCLOUD_INVALID = 13,
  WARN_CLOUD_STALE = 14,
};

inline std::string healthStatusText(const HealthStatus status)
{
  switch (status) {
    case HealthStatus::OK: return "OK";
    case HealthStatus::DEGRADED_CLOUD_TIMEOUT: return "DEGRADED_CLOUD_TIMEOUT";
    case HealthStatus::DEGRADED_TF_FAILED: return "DEGRADED_TF_FAILED";
    case HealthStatus::DEGRADED_EMPTY_CLOUD: return "DEGRADED_EMPTY_CLOUD";
    case HealthStatus::DEGRADED_MAVLINK_ERROR: return "DEGRADED_MAVLINK_ERROR";
    case HealthStatus::DEGRADED_LOW_RATE: return "DEGRADED_LOW_RATE";
    case HealthStatus::DEGRADED_RAW_CLOUD_EMPTY: return "DEGRADED_RAW_CLOUD_EMPTY";
    case HealthStatus::WARN_FILTERED_CLOUD_EMPTY: return "WARN_FILTERED_CLOUD_EMPTY";
    case HealthStatus::DEGRADED_SOURCE_STAMP_ZERO: return "DEGRADED_SOURCE_STAMP_ZERO";
    case HealthStatus::DEGRADED_SOURCE_STAMP_OLD: return "DEGRADED_SOURCE_STAMP_OLD";
    case HealthStatus::DEGRADED_SOURCE_STAMP_REPEATED: return "DEGRADED_SOURCE_STAMP_REPEATED";
    case HealthStatus::DEGRADED_SOURCE_STAMP_BACKWARD: return "DEGRADED_SOURCE_STAMP_BACKWARD";
    case HealthStatus::DEGRADED_SOURCE_STAMP_FUTURE: return "DEGRADED_SOURCE_STAMP_FUTURE";
    case HealthStatus::DEGRADED_POINTCLOUD_INVALID: return "DEGRADED_POINTCLOUD_INVALID";
    case HealthStatus::WARN_CLOUD_STALE: return "WARN_CLOUD_STALE";
  }
  return "UNKNOWN";
}

inline bool isUnknownDistance(const uint16_t distance_cm)
{
  return distance_cm == kUnknownDistanceCm;
}

inline bool isNoObstacleDistance(const uint16_t distance_cm, const uint16_t max_distance_cm)
{
  return distance_cm == static_cast<uint16_t>(max_distance_cm + 1U);
}

inline bool isObstacleDistance(
  const uint16_t distance_cm,
  const uint16_t min_distance_cm,
  const uint16_t max_distance_cm)
{
  return distance_cm >= min_distance_cm && distance_cm <= max_distance_cm;
}

using SectorArray = std::array<uint16_t, kSectorCount>;

}  // namespace aero_halo_360
