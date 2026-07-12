#include "aero_halo_360/temporal_filter.hpp"

#include <algorithm>
#include <cmath>

namespace aero_halo_360
{

TemporalFilter::TemporalFilter(TemporalFilterConfig config)
: config_(config)
{
  reset();
}

SectorArray TemporalFilter::filter(const SectorArray & input)
{
  if (!config_.enable) {
    state_ = input;
    return input;
  }

  SectorArray output = state_;

  for (std::size_t i = 0; i < kSectorCount; ++i) {
    const auto incoming = input[i];
    const auto old = state_[i];

    if (isObstacleDistance(incoming, config_.min_distance_cm, config_.max_distance_cm)) {
      clear_counts_[i] = 0;
      if (!isObstacleDistance(old, config_.min_distance_cm, config_.max_distance_cm) ||
          (incoming < old && config_.approaching_immediate))
      {
        output[i] = incoming;
      } else {
        const double smoothed =
          config_.receding_alpha * static_cast<double>(incoming) +
          (1.0 - config_.receding_alpha) * static_cast<double>(old);
        output[i] = static_cast<uint16_t>(std::lround(std::clamp(
          smoothed,
          static_cast<double>(config_.min_distance_cm),
          static_cast<double>(config_.max_distance_cm))));
      }
      continue;
    }

    if (isNoObstacleDistance(incoming, config_.max_distance_cm)) {
      clear_counts_[i] += 1;
      if (clear_counts_[i] >= config_.clear_frames) {
        output[i] = noObstacleCm();
      } else {
        output[i] = old;
      }
      continue;
    }

    if (isUnknownDistance(incoming)) {
      output[i] = old;
      continue;
    }

    output[i] = old;
  }

  state_ = output;
  return output;
}

void TemporalFilter::reset()
{
  state_.fill(noObstacleCm());
  clear_counts_.fill(0);
}

const SectorArray & TemporalFilter::state() const
{
  return state_;
}

uint16_t TemporalFilter::noObstacleCm() const
{
  return static_cast<uint16_t>(config_.max_distance_cm + 1U);
}

}  // namespace aero_halo_360

