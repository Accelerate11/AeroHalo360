#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "aero_halo_360/pointcloud_validation.hpp"

using aero_halo_360::PointCloudValidationConfig;
using aero_halo_360::validatePointCloud2;

namespace
{

sensor_msgs::msg::PointCloud2 validCloud(const uint32_t width = 2)
{
  sensor_msgs::msg::PointCloud2 message;
  message.height = 1;
  message.width = width;
  message.is_bigendian = false;
  message.point_step = 12;
  message.row_step = width * message.point_step;
  message.data.resize(message.row_step);
  for (uint32_t index = 0; index < 3; ++index) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(1, static_cast<char>('x' + index));
    field.offset = index * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    message.fields.push_back(field);
  }
  return message;
}

}  // namespace

TEST(PointCloudValidation, AcceptsCanonicalXyzFloat32)
{
  EXPECT_TRUE(validatePointCloud2(validCloud(), {}).valid);
}

TEST(PointCloudValidation, RejectsMissingOrWrongFields)
{
  auto missing = validCloud();
  missing.fields.pop_back();
  EXPECT_EQ(validatePointCloud2(missing, {}).reason, "FIELD_MISSING_OR_DUPLICATE_z");

  auto wrong = validCloud();
  wrong.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_EQ(validatePointCloud2(wrong, {}).reason, "FIELD_NOT_FLOAT32_x");
}

TEST(PointCloudValidation, RejectsBigEndianAndTruncatedData)
{
  auto big_endian = validCloud();
  big_endian.is_bigendian = true;
  EXPECT_EQ(validatePointCloud2(big_endian, {}).reason, "BIG_ENDIAN_UNSUPPORTED");

  auto truncated = validCloud();
  truncated.data.pop_back();
  EXPECT_EQ(validatePointCloud2(truncated, {}).reason, "DATA_SIZE_MISMATCH");
}

TEST(PointCloudValidation, EnforcesPointAndByteLimitsBeforeAllocation)
{
  PointCloudValidationConfig config;
  config.max_points = 1;
  EXPECT_EQ(validatePointCloud2(validCloud(2), config).reason, "POINT_COUNT_LIMIT");

  config.max_points = 10;
  config.max_bytes = 12;
  EXPECT_EQ(validatePointCloud2(validCloud(2), config).reason, "DATA_SIZE_LIMIT");
}

TEST(PointCloudValidation, RejectsExcessiveNanAndInfValues)
{
  auto message = validCloud(2);
  const float nan_value = std::numeric_limits<float>::quiet_NaN();
  const float inf_value = std::numeric_limits<float>::infinity();
  std::memcpy(message.data.data(), &nan_value, sizeof(float));
  std::memcpy(message.data.data() + message.point_step + 4, &inf_value, sizeof(float));
  EXPECT_EQ(validatePointCloud2(message, {}).reason, "NONFINITE_XYZ_LIMIT");

  PointCloudValidationConfig tolerant;
  tolerant.max_nonfinite_fraction = 1.0;
  const auto accepted = validatePointCloud2(message, tolerant);
  EXPECT_TRUE(accepted.valid);
  EXPECT_EQ(accepted.nonfinite_point_count, 2U);
}

TEST(PointCloudValidation, RejectsInvalidSteps)
{
  auto message = validCloud();
  message.row_step = 4;
  message.data.resize(4);
  EXPECT_EQ(validatePointCloud2(message, {}).reason, "ROW_STEP_TOO_SMALL");

  message = validCloud();
  message.fields[2].offset = message.point_step;
  EXPECT_EQ(validatePointCloud2(message, {}).reason, "FIELD_OUT_OF_POINT_STEP_z");
}
