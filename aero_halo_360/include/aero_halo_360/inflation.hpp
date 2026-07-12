#pragma once

#include <cstdint>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

struct InflationConfig
{
  bool enable{true};
  double vehicle_radius_m{0.45};
  double safety_extra_m{0.25};
  double sector_deg{5.0};
  int max_inflate_bins{36};
  uint16_t min_distance_cm{25};
  uint16_t max_distance_cm{1000};
};

class Inflation
{
public:
  explicit Inflation(InflationConfig config = {});

  SectorArray apply(const SectorArray & input) const;
  int binsForDistance(uint16_t distance_cm) const;
  const InflationConfig & config() const;

private:
  InflationConfig config_;
};

}  // namespace aero_halo_360
