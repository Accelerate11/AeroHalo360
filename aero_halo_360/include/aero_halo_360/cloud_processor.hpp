#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "aero_halo_360/inflation.hpp"
#include "aero_halo_360/sectorizer.hpp"
#include "aero_halo_360/self_mask.hpp"
#include "aero_halo_360/temporal_filter.hpp"
#include "aero_halo_360/watchdog.hpp"

namespace aero_halo_360
{

struct CloudFilterConfig
{
  std::string input_topic{"/livox/lidar"};
  std::string target_frame{"base_link"};
  double min_range_m{0.25};
  double max_range_m{10.0};
  double z_min_m{-0.40};
  double z_max_m{1.20};
  bool voxel_enable{true};
  double voxel_leaf_size_m{0.05};
  bool radius_outlier_enable{true};
  double radius_outlier_radius_m{0.15};
  int radius_outlier_min_neighbors{2};
  double publish_rate_hz{10.0};
  std::string timestamp_mode{"live"};
  int max_sensor_age_ms{700};
  int future_tolerance_ms{100};
  int max_repeated_frames{2};
  std::size_t max_points{1000000};
  std::size_t max_bytes{67108864};
  bool allow_bigendian{false};
  double max_nonfinite_fraction{0.05};
};

}  // namespace aero_halo_360
