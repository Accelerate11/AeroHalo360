#include "aero_halo_360/sectorizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aero_halo_360
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

uint16_t metersToCentimeters(const double meters)
{
  const auto cm = static_cast<int>(std::lround(meters * 100.0));
  return static_cast<uint16_t>(std::clamp(cm, 0, static_cast<int>(kUnknownDistanceCm - 1)));
}
}  // namespace

Sectorizer::Sectorizer(SectorizerConfig config)
: config_(config)
{
}

SectorArray Sectorizer::compute(const std::vector<PointXYZ> & points) const
{
  const int bin_count = std::max(
    1,
    static_cast<int>(std::ceil(
      (config_.max_range_m - config_.min_range_m) /
      config_.radial_bin_width_m)));

  std::vector<std::vector<std::vector<double>>> bins(
    kSectorCount, std::vector<std::vector<double>>(bin_count));

  for (const auto & point : points) {
    const double measured_range_m = std::hypot(point.x, point.y);
    if (measured_range_m > config_.max_range_m) {
      continue;
    }

    // 小于最小量程但仍有方向信息的点，钳位为最近障碍，绝不能等价于净空。
    const double effective_range_m =
      std::max(config_.min_range_m, measured_range_m);
    const int sector = sectorForPoint(point);
    const int bin = static_cast<int>(std::floor(
      (effective_range_m - config_.min_range_m) /
      config_.radial_bin_width_m));
    const int bounded_bin = std::clamp(bin, 0, bin_count - 1);
    bins[sector][bounded_bin].push_back(effective_range_m);
  }

  SectorArray output;
  output.fill(noObstacleCm());

  for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
    for (int bin = 0; bin < bin_count; ++bin) {
      auto & samples = bins[sector][bin];
      if (samples.empty()) {
        continue;
      }

      const double bin_range_m = config_.min_range_m +
        (static_cast<double>(bin) + 0.5) * config_.radial_bin_width_m;
      if (static_cast<int>(samples.size()) < requiredPointCount(bin_range_m)) {
        continue;
      }

      std::sort(samples.begin(), samples.end());
      const auto index = static_cast<std::size_t>(std::floor(
        config_.percentile * static_cast<double>(samples.size() - 1)));
      output[sector] = metersToCentimeters(samples[index]);
      break;
    }
  }
  return output;
}

int Sectorizer::sectorForPoint(const PointXYZ & point) const
{
  double theta_deg = std::atan2(-point.y, point.x) * 180.0 / kPi;
  theta_deg = std::fmod(theta_deg + 360.0, 360.0);
  return static_cast<int>(std::lround(theta_deg / config_.sector_deg)) %
         static_cast<int>(kSectorCount);
}

int Sectorizer::requiredPointCount(const double range_m) const
{
  if (range_m <= config_.near_range_m) {
    return config_.near_min_points;
  }
  if (range_m <= config_.mid_range_m) {
    return config_.mid_min_points;
  }
  return config_.far_min_points;
}

const SectorizerConfig & Sectorizer::config() const
{
  return config_;
}

uint16_t Sectorizer::noObstacleCm() const
{
  return static_cast<uint16_t>(metersToCentimeters(config_.max_range_m) + 1U);
}

}  // namespace aero_halo_360
