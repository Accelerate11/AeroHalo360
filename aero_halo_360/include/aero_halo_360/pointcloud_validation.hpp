#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace aero_halo_360
{

struct PointCloudValidationConfig
{
  std::size_t max_points{1000000};
  std::size_t max_bytes{67108864};
  bool allow_bigendian{false};
  double max_nonfinite_fraction{0.05};
};

struct PointCloudValidationResult
{
  bool valid{false};
  std::size_t point_count{0};
  std::size_t finite_point_count{0};
  std::size_t nonfinite_point_count{0};
  std::string reason;
};

PointCloudValidationResult validatePointCloud2(
  const sensor_msgs::msg::PointCloud2 & message,
  const PointCloudValidationConfig & config);

}  // namespace aero_halo_360
