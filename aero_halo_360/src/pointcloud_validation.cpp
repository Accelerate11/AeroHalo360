#include "aero_halo_360/pointcloud_validation.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

namespace aero_halo_360
{
namespace
{

bool multiplyWouldOverflow(const std::size_t left, const std::size_t right)
{
  return right != 0 && left > std::numeric_limits<std::size_t>::max() / right;
}

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  const sensor_msgs::msg::PointField * found = nullptr;
  for (const auto & field : message.fields) {
    if (field.name == name) {
      if (found != nullptr) {
        return nullptr;
      }
      found = &field;
    }
  }
  return found;
}

}  // namespace

PointCloudValidationResult validatePointCloud2(
  const sensor_msgs::msg::PointCloud2 & message,
  const PointCloudValidationConfig & config)
{
  PointCloudValidationResult result;
  if (message.is_bigendian && !config.allow_bigendian) {
    result.reason = "BIG_ENDIAN_UNSUPPORTED";
    return result;
  }
  if (message.point_step == 0) {
    result.reason = "POINT_STEP_ZERO";
    return result;
  }

  for (const auto * name : {"x", "y", "z"}) {
    const auto * field = findField(message, name);
    if (field == nullptr) {
      result.reason = std::string("FIELD_MISSING_OR_DUPLICATE_") + name;
      return result;
    }
    if (field->datatype != sensor_msgs::msg::PointField::FLOAT32 || field->count != 1) {
      result.reason = std::string("FIELD_NOT_FLOAT32_") + name;
      return result;
    }
    if (field->offset > message.point_step ||
      message.point_step - field->offset < sizeof(float))
    {
      result.reason = std::string("FIELD_OUT_OF_POINT_STEP_") + name;
      return result;
    }
  }

  if (multiplyWouldOverflow(message.width, message.height)) {
    result.reason = "POINT_COUNT_OVERFLOW";
    return result;
  }
  result.point_count =
    static_cast<std::size_t>(message.width) * static_cast<std::size_t>(message.height);
  if (result.point_count > config.max_points) {
    result.reason = "POINT_COUNT_LIMIT";
    return result;
  }

  if (multiplyWouldOverflow(message.width, message.point_step)) {
    result.reason = "ROW_STEP_OVERFLOW";
    return result;
  }
  const std::size_t minimum_row_step =
    static_cast<std::size_t>(message.width) * static_cast<std::size_t>(message.point_step);
  if (message.row_step < minimum_row_step) {
    result.reason = "ROW_STEP_TOO_SMALL";
    return result;
  }
  if (multiplyWouldOverflow(message.row_step, message.height)) {
    result.reason = "DATA_SIZE_OVERFLOW";
    return result;
  }
  const std::size_t expected_bytes =
    static_cast<std::size_t>(message.row_step) * static_cast<std::size_t>(message.height);
  if (expected_bytes > config.max_bytes) {
    result.reason = "DATA_SIZE_LIMIT";
    return result;
  }
  if (message.data.size() != expected_bytes) {
    result.reason = "DATA_SIZE_MISMATCH";
    return result;
  }

  const auto * field_x = findField(message, "x");
  const auto * field_y = findField(message, "y");
  const auto * field_z = findField(message, "z");
  for (std::size_t row = 0; row < message.height; ++row) {
    for (std::size_t column = 0; column < message.width; ++column) {
      const std::size_t base = row * message.row_step + column * message.point_step;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      std::memcpy(&x, message.data.data() + base + field_x->offset, sizeof(float));
      std::memcpy(&y, message.data.data() + base + field_y->offset, sizeof(float));
      std::memcpy(&z, message.data.data() + base + field_z->offset, sizeof(float));
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        result.finite_point_count += 1;
      } else {
        result.nonfinite_point_count += 1;
      }
    }
  }
  if (result.point_count > 0) {
    const double nonfinite_fraction = static_cast<double>(result.nonfinite_point_count) /
      static_cast<double>(result.point_count);
    if (nonfinite_fraction > config.max_nonfinite_fraction) {
      result.reason = "NONFINITE_XYZ_LIMIT";
      return result;
    }
  }

  result.valid = true;
  result.reason = "OK";
  return result;
}

}  // namespace aero_halo_360
