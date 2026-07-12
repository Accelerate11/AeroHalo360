#include <chrono>

#include <gtest/gtest.h>

#include "aero_halo_360/watchdog.hpp"

using aero_halo_360::DegradedMode;
using aero_halo_360::HealthStatus;
using aero_halo_360::Watchdog;
using aero_halo_360::WatchdogConfig;

TEST(Watchdog, StartupAndConsecutiveLossFailClosed)
{
  WatchdogConfig config;
  config.cloud_warn_timeout_ms = 400;
  config.cloud_fail_timeout_ms = 800;
  config.recovery_healthy_frames = 1;
  config.degraded_mode = DegradedMode::VirtualWall;
  config.virtual_wall_cm = 80;
  Watchdog watchdog(config);

  const auto now = Watchdog::Clock::now();
  EXPECT_TRUE(watchdog.evaluate(now).degraded);

  watchdog.markCloud(now);
  EXPECT_FALSE(watchdog.evaluate(now + std::chrono::milliseconds(100)).degraded);

  const auto timed_out = watchdog.evaluate(now + std::chrono::milliseconds(801));
  EXPECT_TRUE(timed_out.degraded);
  EXPECT_EQ(timed_out.status, HealthStatus::DEGRADED_CLOUD_TIMEOUT);
  EXPECT_EQ(watchdog.degradedDistances()[0], 80);
}

TEST(Watchdog, A332MillisecondJitterDoesNotFailClosed)
{
  WatchdogConfig config;
  config.cloud_warn_timeout_ms = 400;
  config.cloud_fail_timeout_ms = 800;
  config.recovery_healthy_frames = 1;
  Watchdog watchdog(config);

  const auto now = Watchdog::Clock::now();
  watchdog.markCloud(now);
  const auto result = watchdog.evaluate(now + std::chrono::milliseconds(332));
  EXPECT_FALSE(result.degraded);
  EXPECT_EQ(result.status, HealthStatus::OK);
}

TEST(Watchdog, WarnsBeforeFailClosed)
{
  WatchdogConfig config;
  config.cloud_warn_timeout_ms = 400;
  config.cloud_fail_timeout_ms = 800;
  config.recovery_healthy_frames = 1;
  Watchdog watchdog(config);

  const auto now = Watchdog::Clock::now();
  watchdog.markCloud(now);
  watchdog.evaluate(now);
  const auto warning = watchdog.evaluate(now + std::chrono::milliseconds(500));
  EXPECT_FALSE(warning.degraded);
  EXPECT_EQ(warning.status, HealthStatus::WARN_CLOUD_STALE);
}

TEST(Watchdog, RequiresNewHealthyFramesForRecovery)
{
  WatchdogConfig config;
  config.cloud_warn_timeout_ms = 400;
  config.cloud_fail_timeout_ms = 800;
  config.recovery_healthy_frames = 3;
  Watchdog watchdog(config);

  const auto now = Watchdog::Clock::now();
  watchdog.markCloud(now);
  EXPECT_TRUE(watchdog.evaluate(now).degraded);
  EXPECT_TRUE(watchdog.evaluate(now + std::chrono::milliseconds(10)).degraded);

  watchdog.markCloud(now + std::chrono::milliseconds(20));
  EXPECT_TRUE(watchdog.evaluate(now + std::chrono::milliseconds(20)).degraded);
  watchdog.markCloud(now + std::chrono::milliseconds(40));
  const auto recovered = watchdog.evaluate(now + std::chrono::milliseconds(40));
  EXPECT_FALSE(recovered.degraded);
  EXPECT_EQ(recovered.status, HealthStatus::OK);

  EXPECT_TRUE(watchdog.evaluate(now + std::chrono::milliseconds(900)).degraded);
  watchdog.markCloud(now + std::chrono::milliseconds(910));
  EXPECT_TRUE(watchdog.evaluate(now + std::chrono::milliseconds(910)).degraded);
  watchdog.markCloud(now + std::chrono::milliseconds(920));
  EXPECT_TRUE(watchdog.evaluate(now + std::chrono::milliseconds(920)).degraded);
  watchdog.markCloud(now + std::chrono::milliseconds(930));
  EXPECT_FALSE(watchdog.evaluate(now + std::chrono::milliseconds(930)).degraded);
}

TEST(Watchdog, MonotonicBackwardEvaluationDoesNotCreateFalseTimeout)
{
  WatchdogConfig config;
  config.recovery_healthy_frames = 1;
  Watchdog watchdog(config);
  const auto now = Watchdog::Clock::now();
  watchdog.markCloud(now);
  watchdog.evaluate(now);
  const auto result = watchdog.evaluate(now - std::chrono::seconds(1));
  EXPECT_FALSE(result.degraded);
  EXPECT_EQ(result.cloud_age_ms, 0.0);
}

TEST(Watchdog, ReportsTransitions)
{
  WatchdogConfig config;
  config.recovery_healthy_frames = 1;
  Watchdog watchdog(config);
  const auto now = Watchdog::Clock::now();
  watchdog.markCloud(now);
  watchdog.evaluate(now);
  const auto result = watchdog.evaluate(now + std::chrono::seconds(2));
  EXPECT_GE(result.transition_count, 1U);
  EXPECT_NE(result.transition_reason.find("DEGRADED_CLOUD_TIMEOUT"), std::string::npos);
}

TEST(Watchdog, SupportsUnknownAndMaxPlusOneModes)
{
  WatchdogConfig unknown_config;
  unknown_config.degraded_mode = DegradedMode::Unknown;
  Watchdog unknown(unknown_config);
  EXPECT_EQ(unknown.degradedDistances()[0], 65535);

  WatchdogConfig clear_config;
  clear_config.degraded_mode = DegradedMode::MaxPlusOne;
  clear_config.max_distance_cm = 1000;
  Watchdog clear(clear_config);
  EXPECT_EQ(clear.degradedDistances()[0], 1001);
}

TEST(Watchdog, EmptyCloudPoliciesAreExplicit)
{
  WatchdogConfig warning_config;
  warning_config.recovery_healthy_frames = 1;
  warning_config.empty_cloud_limit = 2;
  warning_config.filtered_empty_is_clear = true;
  warning_config.filtered_empty_warn_only = true;
  Watchdog warning(warning_config);

  const auto now = Watchdog::Clock::now();
  warning.markCloud(now);
  warning.markRawCloudSize(100);
  warning.markFilteredCloudSize(10);
  EXPECT_FALSE(warning.evaluate(now).degraded);
  warning.markFilteredCloudSize(0);
  warning.markFilteredCloudSize(0);
  const auto warning_result = warning.evaluate(now);
  EXPECT_FALSE(warning_result.degraded);
  EXPECT_EQ(warning_result.status, HealthStatus::WARN_FILTERED_CLOUD_EMPTY);

  WatchdogConfig flight_config = warning_config;
  flight_config.filtered_empty_is_clear = false;
  flight_config.filtered_empty_warn_only = false;
  Watchdog flight(flight_config);
  flight.markCloud(now);
  flight.markRawCloudSize(100);
  flight.markFilteredCloudSize(0);
  flight.markFilteredCloudSize(0);
  EXPECT_EQ(flight.evaluate(now).status, HealthStatus::DEGRADED_EMPTY_CLOUD);
}

TEST(Watchdog, RawEmptyAndInvalidSourceFailClosed)
{
  WatchdogConfig config;
  config.recovery_healthy_frames = 1;
  config.raw_cloud_empty_limit = 2;
  Watchdog watchdog(config);
  const auto now = Watchdog::Clock::now();
  watchdog.markCloud(now);
  watchdog.markRawCloudSize(0);
  watchdog.markRawCloudSize(0);
  EXPECT_EQ(watchdog.evaluate(now).status, HealthStatus::DEGRADED_RAW_CLOUD_EMPTY);

  watchdog.markRawCloudSize(10);
  watchdog.markSourceInvalid(HealthStatus::DEGRADED_POINTCLOUD_INVALID);
  EXPECT_EQ(watchdog.evaluate(now).status, HealthStatus::DEGRADED_POINTCLOUD_INVALID);
}
