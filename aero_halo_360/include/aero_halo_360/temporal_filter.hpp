#pragma once

#include <array>
#include <cstdint>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

struct TemporalFilterConfig
{
  bool enable{true};
  uint16_t min_distance_cm{25};
  uint16_t max_distance_cm{1000};
  int clear_frames{3};
  double receding_alpha{0.4};
  bool approaching_immediate{true};
};

class TemporalFilter
{
public:
  explicit TemporalFilter(TemporalFilterConfig config = {});

  SectorArray filter(const SectorArray & input);
  void reset();
  const SectorArray & state() const;

private:
  TemporalFilterConfig config_;
  SectorArray state_{};
  std::array<int, kSectorCount> clear_counts_{};

  uint16_t noObstacleCm() const;
};

}  // namespace aero_halo_360

