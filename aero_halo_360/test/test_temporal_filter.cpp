#include <gtest/gtest.h>

#include "aero_halo_360/temporal_filter.hpp"

using aero_halo_360::SectorArray;
using aero_halo_360::TemporalFilter;
using aero_halo_360::TemporalFilterConfig;

TEST(TemporalFilter, ApproachingObstacleUpdatesImmediately)
{
  TemporalFilterConfig config;
  config.clear_frames = 3;
  config.receding_alpha = 0.4;
  TemporalFilter filter(config);

  SectorArray input;
  input.fill(1001);
  input[0] = 300;
  EXPECT_EQ(filter.filter(input)[0], 300);

  input[0] = 150;
  EXPECT_EQ(filter.filter(input)[0], 150);
}

TEST(TemporalFilter, RecedingObstacleMovesOutSmoothly)
{
  TemporalFilterConfig config;
  config.receding_alpha = 0.4;
  TemporalFilter filter(config);

  SectorArray input;
  input.fill(1001);
  input[0] = 200;
  EXPECT_EQ(filter.filter(input)[0], 200);

  input[0] = 300;
  EXPECT_EQ(filter.filter(input)[0], 240);
}

TEST(TemporalFilter, ClearsOnlyAfterConfiguredFrames)
{
  TemporalFilterConfig config;
  config.clear_frames = 2;
  TemporalFilter filter(config);

  SectorArray input;
  input.fill(1001);
  input[0] = 200;
  EXPECT_EQ(filter.filter(input)[0], 200);

  input[0] = 1001;
  EXPECT_EQ(filter.filter(input)[0], 200);
  EXPECT_EQ(filter.filter(input)[0], 1001);
}
