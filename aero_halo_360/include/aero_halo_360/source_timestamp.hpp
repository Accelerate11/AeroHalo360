#pragma once

#include <cstdint>
#include <string>

#include "aero_halo_360/types.hpp"

namespace aero_halo_360
{

struct SourceTimestampConfig
{
  std::string mode{"live"};
  int max_sensor_age_ms{700};
  int future_tolerance_ms{100};
  int max_repeated_frames{2};
};

struct SourceTimestampResult
{
  bool accepted{false};
  HealthStatus status{HealthStatus::OK};
  uint64_t sequence{0};
  double age_ms{0.0};
};

class SourceTimestampGuard
{
public:
  explicit SourceTimestampGuard(SourceTimestampConfig config = {});

  SourceTimestampResult evaluate(int64_t stamp_ns, int64_t now_ns);
  void reset();
  const SourceTimestampConfig & config() const;

private:
  SourceTimestampConfig config_;
  bool has_stamp_{false};
  int64_t last_stamp_ns_{0};
  int consecutive_repeated_{0};
  uint64_t sequence_{0};
};

}  // namespace aero_halo_360
