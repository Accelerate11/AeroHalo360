#include "aero_halo_360/inflation.hpp"

#include <algorithm>
#include <cmath>

namespace aero_halo_360
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaximumSafeBins = static_cast<int>(kSectorCount / 2U);
}  // namespace

Inflation::Inflation(InflationConfig config)
: config_(config)
{
}

SectorArray Inflation::apply(const SectorArray & input) const
{
  if (!config_.enable) {
    return input;
  }

  SectorArray output = input;
  for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
    const auto distance_cm = input[sector];
    if (!isObstacleDistance(distance_cm, config_.min_distance_cm, config_.max_distance_cm)) {
      continue;
    }

    const int bins = binsForDistance(distance_cm);
    for (int offset = -bins; offset <= bins; ++offset) {
      const auto target =
        (static_cast<int>(sector) + offset + static_cast<int>(kSectorCount)) %
        static_cast<int>(kSectorCount);
      if (!isObstacleDistance(output[target], config_.min_distance_cm, config_.max_distance_cm) ||
          distance_cm < output[target])
      {
        output[target] = distance_cm;
      }
    }
  }
  return output;
}

int Inflation::binsForDistance(const uint16_t distance_cm) const
{
  if (!isObstacleDistance(distance_cm, config_.min_distance_cm, config_.max_distance_cm)) {
    return 0;
  }

  const double distance_m = std::max(0.01, static_cast<double>(distance_cm) / 100.0);
  const double inflate_angle_rad =
    std::atan((config_.vehicle_radius_m + config_.safety_extra_m) / distance_m);
  const double inflate_angle_deg = inflate_angle_rad * 180.0 / kPi;
  const int geometric_bins =
    static_cast<int>(std::ceil(inflate_angle_deg / config_.sector_deg));
  const int configured_limit = config_.max_inflate_bins <= 0 ?
    kMaximumSafeBins : std::min(config_.max_inflate_bins, kMaximumSafeBins);
  return std::clamp(geometric_bins, 0, configured_limit);
}

const InflationConfig & Inflation::config() const
{
  return config_;
}

}  // namespace aero_halo_360
