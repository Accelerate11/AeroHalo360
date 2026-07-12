#include <gtest/gtest.h>

#include "aero_halo_360/self_mask.hpp"

using aero_halo_360::MaskBox;
using aero_halo_360::PointXYZ;
using aero_halo_360::SelfMask;

TEST(SelfMask, RemovesPointsInsideConfiguredBoxes)
{
  SelfMask mask;
  mask.addBox(MaskBox{"body", PointXYZ{-0.5, -0.5, -0.2}, PointXYZ{0.5, 0.5, 0.2}});

  EXPECT_TRUE(mask.isMasked(PointXYZ{0.0, 0.0, 0.0}));
  EXPECT_FALSE(mask.isMasked(PointXYZ{1.0, 0.0, 0.0}));

  const auto filtered = mask.filter({PointXYZ{0.0, 0.0, 0.0}, PointXYZ{1.0, 0.0, 0.0}});
  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_DOUBLE_EQ(filtered[0].x, 1.0);
}
TEST(SelfMask, FullyMaskedNonEmptyCloudRemainsDistinguishableFromRawEmpty)
{
  SelfMask mask;
  mask.addBox(MaskBox{
    "bad_calibration",
    PointXYZ{-10.0, -10.0, -10.0},
    PointXYZ{10.0, 10.0, 10.0}});
  const std::vector<PointXYZ> raw{
    PointXYZ{1.0, 0.0, 0.0},
    PointXYZ{2.0, 0.0, 0.0}};
  const auto filtered = mask.filter(raw);
  EXPECT_FALSE(raw.empty());
  EXPECT_TRUE(filtered.empty());
}
