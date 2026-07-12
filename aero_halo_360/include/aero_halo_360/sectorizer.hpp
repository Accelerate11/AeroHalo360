#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

struct SectorizerConfig
{
  double sector_deg{5.0};
  double min_range_m{0.25};
  double max_range_m{10.0};
  double radial_bin_width_m{0.25};
  double percentile{0.20};
  double near_range_m{3.0};
  double mid_range_m{6.0};
  int near_min_points{3};
  int mid_min_points{2};
  int far_min_points{1};
};

class Sectorizer
{
public:
  explicit Sectorizer(SectorizerConfig config = {});

  SectorArray compute(const std::vector<PointXYZ> & points) const;
  int sectorForPoint(const PointXYZ & point) const;
  int requiredPointCount(double range_m) const;
  const SectorizerConfig & config() const;

private:
  SectorizerConfig config_;
  uint16_t noObstacleCm() const;
};

}  // namespace aero_halo_360

