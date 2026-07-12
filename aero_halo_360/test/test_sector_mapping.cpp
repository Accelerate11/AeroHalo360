#include <cmath>

#include <gtest/gtest.h>

#include "aero_halo_360/sectorizer.hpp"

using aero_halo_360::PointXYZ;
using aero_halo_360::Sectorizer;
using aero_halo_360::SectorizerConfig;

namespace
{
SectorizerConfig testConfig()
{
  SectorizerConfig config;
  config.min_range_m = 0.25;
  config.max_range_m = 10.0;
  config.near_min_points = 1;
  config.mid_min_points = 1;
  config.far_min_points = 1;
  return config;
}
}  // namespace

TEST(Sectorizer, MapsBodyFrameDirectionsToMavlinkSectors)
{
  Sectorizer sectorizer(testConfig());
  EXPECT_EQ(sectorizer.sectorForPoint(PointXYZ{2.0, 0.0, 0.0}), 0);
  EXPECT_EQ(sectorizer.sectorForPoint(PointXYZ{0.0, -2.0, 0.0}), 18);
  EXPECT_EQ(sectorizer.sectorForPoint(PointXYZ{-2.0, 0.0, 0.0}), 36);
  EXPECT_EQ(sectorizer.sectorForPoint(PointXYZ{0.0, 2.0, 0.0}), 54);
}

TEST(Sectorizer, ExtractsNearestConfirmedRadialBin)
{
  Sectorizer sectorizer(testConfig());
  const auto sectors = sectorizer.compute({PointXYZ{2.0, 0.0, 0.0}});
  EXPECT_NEAR(sectors[0], 200, 1);
  EXPECT_EQ(sectors[18], 1001);
}
TEST(Sectorizer, ClampsTooCloseObstacleToMinimumDistance)
{
  Sectorizer sectorizer(testConfig());
  const auto sectors = sectorizer.compute({PointXYZ{0.10, 0.0, 0.0}});
  EXPECT_EQ(sectors[0], 25);
  EXPECT_EQ(sectors[18], 1001);
}

TEST(Sectorizer, PreservesDirectionForTooCloseObstacles)
{
  Sectorizer sectorizer(testConfig());
  EXPECT_EQ(sectorizer.compute({PointXYZ{0.0, -0.10, 0.0}})[18], 25);
  EXPECT_EQ(sectorizer.compute({PointXYZ{-0.10, 0.0, 0.0}})[36], 25);
  EXPECT_EQ(sectorizer.compute({PointXYZ{0.0, 0.10, 0.0}})[54], 25);
}
TEST(Sectorizer, HandlesApproachAndWithdrawalAcrossMinimumRange)
{
  Sectorizer sectorizer(testConfig());
  EXPECT_EQ(sectorizer.compute({PointXYZ{0.10, 0.0, 0.0}})[0], 25);
  EXPECT_EQ(sectorizer.compute({PointXYZ{0.20, 0.0, 0.0}})[0], 25);
  EXPECT_EQ(sectorizer.compute({PointXYZ{0.24, 0.0, 0.0}})[0], 25);
  EXPECT_NEAR(sectorizer.compute({PointXYZ{0.30, 0.0, 0.0}})[0], 30, 1);
  EXPECT_NEAR(sectorizer.compute({PointXYZ{0.40, 0.0, 0.0}})[0], 40, 1);
}

TEST(Sectorizer, DetectsThinPoleAtEverySectorFromPointThreeToOnePointFiveMeters)
{
  Sectorizer sectorizer(testConfig());
  constexpr double kPi = 3.14159265358979323846;
  for (const double range_m : {0.30, 0.50, 1.00, 1.50}) {
    for (int sector = 0; sector < 72; ++sector) {
      const double angle = static_cast<double>(sector) * 5.0 * kPi / 180.0;
      const PointXYZ point{
        range_m * std::cos(angle),
        -range_m * std::sin(angle),
        0.0};
      const auto output = sectorizer.compute({point});
      EXPECT_NEAR(output[static_cast<std::size_t>(sector)], range_m * 100.0, 1.0)
        << "range=" << range_m << " sector=" << sector;
    }
  }
}
