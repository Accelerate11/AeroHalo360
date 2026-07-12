#include <gtest/gtest.h>

#include "aero_halo_360/source_timestamp.hpp"

using aero_halo_360::HealthStatus;
using aero_halo_360::SourceTimestampConfig;
using aero_halo_360::SourceTimestampGuard;

TEST(SourceTimestampGuard, AcceptsMonotonicLiveFrames)
{
  SourceTimestampGuard guard;
  auto first = guard.evaluate(1000000000LL, 1100000000LL);
  auto second = guard.evaluate(1200000000LL, 1250000000LL);
  EXPECT_TRUE(first.accepted);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(first.sequence, 1U);
  EXPECT_EQ(second.sequence, 2U);
}

TEST(SourceTimestampGuard, RejectsZeroOldAndFutureStamps)
{
  SourceTimestampConfig config;
  config.max_sensor_age_ms = 700;
  config.future_tolerance_ms = 100;
  SourceTimestampGuard guard(config);

  EXPECT_EQ(
    guard.evaluate(0, 1000000000LL).status,
    HealthStatus::DEGRADED_SOURCE_STAMP_ZERO);
  EXPECT_EQ(
    guard.evaluate(1000000000LL, 1800000000LL).status,
    HealthStatus::DEGRADED_SOURCE_STAMP_OLD);
  EXPECT_EQ(
    guard.evaluate(1200000000LL, 1000000000LL).status,
    HealthStatus::DEGRADED_SOURCE_STAMP_FUTURE);
}

TEST(SourceTimestampGuard, RejectsFrozenAndBackwardLiveFrames)
{
  SourceTimestampConfig config;
  config.max_repeated_frames = 2;
  SourceTimestampGuard guard(config);

  EXPECT_TRUE(guard.evaluate(1000000000LL, 1000000000LL).accepted);
  EXPECT_FALSE(guard.evaluate(1000000000LL, 1010000000LL).accepted);
  const auto frozen = guard.evaluate(1000000000LL, 1020000000LL);
  EXPECT_FALSE(frozen.accepted);
  EXPECT_EQ(frozen.status, HealthStatus::DEGRADED_SOURCE_STAMP_REPEATED);

  const auto backward = guard.evaluate(900000000LL, 1030000000LL);
  EXPECT_FALSE(backward.accepted);
  EXPECT_EQ(backward.status, HealthStatus::DEGRADED_SOURCE_STAMP_BACKWARD);
}

TEST(SourceTimestampGuard, ReplayModeExplicitlyAllowsBackwardStamp)
{
  SourceTimestampConfig config;
  config.mode = "replay";
  SourceTimestampGuard guard(config);

  EXPECT_TRUE(guard.evaluate(2000000000LL, 10000000000LL).accepted);
  const auto replayed = guard.evaluate(1000000000LL, 11000000000LL);
  EXPECT_TRUE(replayed.accepted);
  EXPECT_EQ(replayed.sequence, 2U);
  EXPECT_DOUBLE_EQ(replayed.age_ms, 0.0);
}
