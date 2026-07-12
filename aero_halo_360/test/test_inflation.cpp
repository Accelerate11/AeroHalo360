#include <gtest/gtest.h>

#include "aero_halo_360/inflation.hpp"

using aero_halo_360::Inflation;
using aero_halo_360::InflationConfig;
using aero_halo_360::SectorArray;

TEST(Inflation, ExpandsObstacleAcrossNeighborSectorsWithWraparound)
{
  InflationConfig config;
  config.vehicle_radius_m = 0.45;
  config.safety_extra_m = 0.25;
  config.max_inflate_bins = 6;
  Inflation inflation(config);

  SectorArray input;
  input.fill(1001);
  input[0] = 200;

  const auto output = inflation.apply(input);
  EXPECT_EQ(output[0], 200);
  EXPECT_EQ(output[1], 200);
  EXPECT_EQ(output[71], 200);
  EXPECT_EQ(output[4], 200);
  EXPECT_EQ(output[68], 200);
  EXPECT_EQ(output[5], 1001);
}

TEST(Inflation, DoesNotInflateUnknownOrClearDirections)
{
  Inflation inflation;
  SectorArray input;
  input.fill(1001);
  input[0] = 65535;
  const auto output = inflation.apply(input);
  EXPECT_EQ(output[1], 1001);
  EXPECT_EQ(output[71], 1001);
}
TEST(Inflation, DoesNotClipRequiredNearObstacleGeometry)
{
  InflationConfig config;
  config.vehicle_radius_m = 0.45;
  config.safety_extra_m = 0.25;
  config.max_inflate_bins = 36;
  Inflation inflation(config);

  EXPECT_GE(inflation.binsForDistance(25), 15);

  SectorArray input;
  input.fill(1001);
  input[0] = 25;
  const auto output = inflation.apply(input);
  const int bins = inflation.binsForDistance(25);
  EXPECT_EQ(output[static_cast<std::size_t>(bins)], 25);
  EXPECT_EQ(output[72U - static_cast<std::size_t>(bins)], 25);
}
