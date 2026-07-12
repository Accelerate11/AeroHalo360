#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aero_halo_360/cloud_processor.hpp"
#include "aero_halo_360/diagnostics.hpp"
#include "aero_halo_360/msg/sector_distances.hpp"
#include "aero_halo_360/pointcloud_validation.hpp"
#include "aero_halo_360/source_timestamp.hpp"

namespace aero_halo_360
{
namespace
{
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<int>{}(key.x);
    const auto hy = std::hash<int>{}(key.y);
    const auto hz = std::hash<int>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

struct VoxelAccum
{
  PointXYZ sum;
  int count{0};
};

VoxelKey gridKeyForPoint(const PointXYZ & point, const double cell_size_m)
{
  const double cell = std::max(0.005, cell_size_m);
  return VoxelKey{
    static_cast<int>(std::floor(point.x / cell)),
    static_cast<int>(std::floor(point.y / cell)),
    static_cast<int>(std::floor(point.z / cell))};
}

double range2d(const PointXYZ & point)
{
  return std::hypot(point.x, point.y);
}

uint16_t metersToCentimeters(const double meters)
{
  const auto cm = static_cast<int>(std::lround(meters * 100.0));
  return static_cast<uint16_t>(std::clamp(cm, 0, static_cast<int>(kUnknownDistanceCm - 1)));
}

}  // namespace

class CloudProcessorNode : public rclcpp::Node
{
public:
  explicit CloudProcessorNode(const rclcpp::NodeOptions & options)
  : Node("cloud_processor_node", options),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();
    validateParameters();

    sectorizer_ = std::make_unique<Sectorizer>(sectorizer_config_);
    temporal_filter_ = std::make_unique<TemporalFilter>(temporal_config_);
    inflation_ = std::make_unique<Inflation>(inflation_config_);
    watchdog_ = std::make_unique<Watchdog>(watchdog_config_);
    source_timestamp_guard_ = std::make_unique<SourceTimestampGuard>(
      SourceTimestampConfig{
        filter_config_.timestamp_mode,
        filter_config_.max_sensor_age_ms,
        filter_config_.future_tolerance_ms,
        filter_config_.max_repeated_frames});

    latest_distances_.fill(static_cast<uint16_t>(sectorizer_config_.max_range_m * 100.0 + 1.0));

    filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_cloud_topic_, rclcpp::SensorDataQoS());
    sector_pub_ = create_publisher<aero_halo_360::msg::SectorDistances>(
      sector_topic_, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      filter_config_.input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&CloudProcessorNode::onCloud, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, filter_config_.publish_rate_hz));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CloudProcessorNode::onPublishTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "AeroHalo360 点云处理节点已启动: input=%s target_frame=%s sectors=72 publish_rate=%.1f Hz",
      filter_config_.input_topic.c_str(),
      filter_config_.target_frame.c_str(),
      filter_config_.publish_rate_hz);
  }

private:
  template<typename T>
  T parameterOr(const std::string & name, const T & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void loadParameters()
  {
    filter_config_.input_topic = parameterOr<std::string>("input_topic", "/livox/lidar");
    filter_config_.target_frame = parameterOr<std::string>("target_frame", "base_link");
    filtered_cloud_topic_ =
      parameterOr<std::string>("filtered_cloud_topic", "/aero_halo_360/filtered_cloud");
    sector_topic_ =
      parameterOr<std::string>("sector_topic", "/aero_halo_360/sector_distances");
    marker_topic_ =
      parameterOr<std::string>("marker_topic", "/aero_halo_360/markers");
    diagnostics_topic_ =
      parameterOr<std::string>("diagnostics_topic", "/aero_halo_360/diagnostics");
    publish_filtered_cloud_ = parameterOr<bool>("debug.publish_filtered_cloud", true);
    publish_markers_ = parameterOr<bool>("debug.publish_markers", true);
    publish_diagnostics_ = parameterOr<bool>("debug.publish_diagnostics", true);
    profile_name_ = parameterOr<std::string>("runtime.profile_name", "default");

    filter_config_.min_range_m = parameterOr<double>("range_filter.min_range_m", 0.25);
    filter_config_.max_range_m = parameterOr<double>("range_filter.max_range_m", 10.0);
    filter_config_.z_min_m = parameterOr<double>("height_filter.z_min_m", -0.40);
    filter_config_.z_max_m = parameterOr<double>("height_filter.z_max_m", 1.20);
    filter_config_.voxel_enable = parameterOr<bool>("voxel_filter.enable", true);
    filter_config_.voxel_leaf_size_m = parameterOr<double>("voxel_filter.leaf_size_m", 0.05);
    filter_config_.radius_outlier_enable = parameterOr<bool>("radius_outlier.enable", true);
    filter_config_.radius_outlier_radius_m =
      parameterOr<double>("radius_outlier.radius_m", 0.15);
    filter_config_.radius_outlier_min_neighbors =
      static_cast<int>(parameterOr<int64_t>("radius_outlier.min_neighbors", 2));
    filter_config_.publish_rate_hz = parameterOr<double>("publish_rate_hz", 10.0);
    filter_config_.timestamp_mode =
      parameterOr<std::string>("timestamp.mode", "live");
    filter_config_.max_sensor_age_ms =
      static_cast<int>(parameterOr<int64_t>("timestamp.max_sensor_age_ms", 700));
    filter_config_.future_tolerance_ms =
      static_cast<int>(parameterOr<int64_t>("timestamp.future_tolerance_ms", 100));
    filter_config_.max_repeated_frames =
      static_cast<int>(parameterOr<int64_t>("timestamp.max_repeated_frames", 2));
    filter_config_.max_points = static_cast<std::size_t>(
      parameterOr<int64_t>("pointcloud_limits.max_points", 1000000));
    filter_config_.max_bytes = static_cast<std::size_t>(
      parameterOr<int64_t>("pointcloud_limits.max_bytes", 67108864));
    filter_config_.allow_bigendian =
      parameterOr<bool>("pointcloud_limits.allow_bigendian", false);
    filter_config_.max_nonfinite_fraction =
      parameterOr<double>("pointcloud_limits.max_nonfinite_fraction", 0.05);

    sectorizer_config_.min_range_m = filter_config_.min_range_m;
    sectorizer_config_.max_range_m = filter_config_.max_range_m;
    sectorizer_config_.sector_deg = parameterOr<double>("sectorizer.sector_deg", 5.0);
    sectorizer_config_.radial_bin_width_m =
      parameterOr<double>("sectorizer.radial_bin_width_m", 0.25);
    sectorizer_config_.percentile = parameterOr<double>("sectorizer.percentile", 0.20);
    sectorizer_config_.near_range_m = parameterOr<double>("sectorizer.near_range_m", 3.0);
    sectorizer_config_.mid_range_m = parameterOr<double>("sectorizer.mid_range_m", 6.0);
    sectorizer_config_.near_min_points =
      static_cast<int>(parameterOr<int64_t>("sectorizer.near_min_points", 3));
    sectorizer_config_.mid_min_points =
      static_cast<int>(parameterOr<int64_t>("sectorizer.mid_min_points", 2));
    sectorizer_config_.far_min_points =
      static_cast<int>(parameterOr<int64_t>("sectorizer.far_min_points", 1));

    temporal_config_.enable = parameterOr<bool>("temporal_filter.enable", true);
    temporal_config_.min_distance_cm = metersToCentimeters(filter_config_.min_range_m);
    temporal_config_.max_distance_cm = metersToCentimeters(filter_config_.max_range_m);
    temporal_config_.clear_frames =
      static_cast<int>(parameterOr<int64_t>("temporal_filter.clear_frames", 3));
    temporal_config_.receding_alpha =
      parameterOr<double>("temporal_filter.receding_alpha", 0.4);
    temporal_config_.approaching_immediate =
      parameterOr<bool>("temporal_filter.approaching_immediate", true);

    inflation_config_.enable = parameterOr<bool>("inflation.enable", true);
    inflation_config_.vehicle_radius_m =
      parameterOr<double>("vehicle.radius_m", parameterOr<double>("inflation.vehicle_radius_m", 0.45));
    inflation_config_.safety_extra_m =
      parameterOr<double>("vehicle.safety_extra_m", parameterOr<double>("inflation.safety_extra_m", 0.25));
    inflation_config_.sector_deg = sectorizer_config_.sector_deg;
    inflation_config_.max_inflate_bins =
      static_cast<int>(parameterOr<int64_t>("inflation.max_inflate_bins", 6));
    inflation_config_.min_distance_cm = temporal_config_.min_distance_cm;
    inflation_config_.max_distance_cm = temporal_config_.max_distance_cm;

    watchdog_config_.cloud_warn_timeout_ms =
      static_cast<int>(parameterOr<int64_t>("watchdog.cloud_warn_timeout_ms", 400));
    watchdog_config_.cloud_fail_timeout_ms =
      static_cast<int>(parameterOr<int64_t>("watchdog.cloud_fail_timeout_ms", 800));
    watchdog_config_.recovery_healthy_frames =
      static_cast<int>(parameterOr<int64_t>("watchdog.recovery_healthy_frames", 3));
    degraded_mode_name_ = parameterOr<std::string>(
      "watchdog.degraded_mode", "virtual_wall");
    watchdog_config_.degraded_mode = Watchdog::parseDegradedMode(degraded_mode_name_);
    watchdog_config_.virtual_wall_cm =
      static_cast<uint16_t>(parameterOr<int64_t>("watchdog.virtual_wall_cm", 80));
    watchdog_config_.max_distance_cm = temporal_config_.max_distance_cm;
    watchdog_config_.min_processing_rate_hz =
      parameterOr<double>("watchdog.min_processing_rate_hz", 5.0);
    watchdog_config_.tf_failure_limit =
      static_cast<int>(parameterOr<int64_t>("watchdog.tf_failure_limit", 3));
    watchdog_config_.raw_cloud_empty_limit =
      static_cast<int>(parameterOr<int64_t>("watchdog.raw_cloud_empty_limit", 3));
    watchdog_config_.empty_cloud_limit =
      static_cast<int>(parameterOr<int64_t>("watchdog.empty_cloud_limit", 3));
    watchdog_config_.filtered_empty_is_clear =
      parameterOr<bool>("watchdog.filtered_empty_is_clear", true);
    watchdog_config_.filtered_empty_warn_only =
      parameterOr<bool>("watchdog.filtered_empty_warn_only", true);

    self_mask_.setBoxes(loadSelfMaskBoxes());
  }

  std::vector<MaskBox> loadSelfMaskBoxes()
  {
    const auto names = parameterOr<std::vector<std::string>>(
      "self_mask.names",
      std::vector<std::string>{});

    std::vector<MaskBox> boxes;
    for (const auto & name : names) {
      const std::string min_param = "self_mask." + name + ".min";
      const std::string max_param = "self_mask." + name + ".max";
      const auto min_values = parameterOr<std::vector<double>>(min_param, std::vector<double>{});
      const auto max_values = parameterOr<std::vector<double>>(max_param, std::vector<double>{});
      if (min_values.size() != 3 || max_values.size() != 3) {
        throw std::invalid_argument(
                "self_mask." + name + ".min/max 必须各包含 3 个数值");
      }
      if (min_values[0] >= max_values[0] ||
        min_values[1] >= max_values[1] ||
        min_values[2] >= max_values[2])
      {
        throw std::invalid_argument(
                "self_mask." + name + " 必须满足每一轴 min < max");
      }
      boxes.push_back(MaskBox{
        name,
        PointXYZ{min_values[0], min_values[1], min_values[2]},
        PointXYZ{max_values[0], max_values[1], max_values[2]}});
    }
    return boxes;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_input_frame_ = msg->header.frame_id;
    const auto validation = validatePointCloud2(
      *msg, PointCloudValidationConfig{
        filter_config_.max_points,
        filter_config_.max_bytes,
        filter_config_.allow_bigendian,
        filter_config_.max_nonfinite_fraction});
    if (!validation.valid) {
      rejected_pointcloud_count_ += 1;
      last_pointcloud_reject_reason_ = validation.reason;
      watchdog_->markSourceInvalid(HealthStatus::DEGRADED_POINTCLOUD_INVALID);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "拒绝无效 PointCloud2: %s", validation.reason.c_str());
      return;
    }

    const auto start = steady_clock::now();
    watchdog_->markCloud(start);
    if (!validateSourceTimestamp(*msg)) {
      return;
    }

    std::vector<PointXYZ> points;
    try {
      points = readAndTransformCloud(*msg);
      watchdog_->markRawCloudSize(points.size());
      watchdog_->markTfSuccess();
      consecutive_tf_failures_ = 0;
    } catch (const std::exception & error) {
      watchdog_->markTfFailure();
      consecutive_tf_failures_ += 1;
      last_tf_error_ = error.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "点云 TF 转换失败: %s", error.what());
      return;
    }

    auto filtered = applyPointFilters(points);
    watchdog_->markFilteredCloudSize(filtered.size());

    if (publish_filtered_cloud_) {
      publishFilteredCloud(filtered, msg->header.stamp);
    }
    const auto raw_sectors = sectorizer_->compute(filtered);
    const auto temporal = temporal_filter_->filter(raw_sectors);
    const auto inflated = inflation_->apply(temporal);

    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_distances_ = inflated;
      latest_header_.stamp = msg->header.stamp;
      latest_header_.frame_id = filter_config_.target_frame;
      latest_filtered_points_ = filtered.size();
    }

    if (publish_markers_) {
      publishMarkers(inflated, msg->header.stamp);
    }

    const auto finish = steady_clock::now();
    updateProcessingRate(start, finish);
  }

  std::vector<PointXYZ> readAndTransformCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const bool needs_transform =
      !filter_config_.target_frame.empty() &&
      msg.header.frame_id != filter_config_.target_frame;

    geometry_msgs::msg::TransformStamped transform;
    tf2::Matrix3x3 rotation;
    PointXYZ translation;

    if (needs_transform) {
      transform = tf_buffer_->lookupTransform(
        filter_config_.target_frame,
        msg.header.frame_id,
        msg.header.stamp,
        std::chrono::milliseconds(50));

      const auto & q_msg = transform.transform.rotation;
      tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      rotation = tf2::Matrix3x3(q);
      translation = PointXYZ{
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z};
    } else {
      rotation.setIdentity();
      translation = PointXYZ{};
    }

    std::vector<PointXYZ> points;
    points.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (needs_transform) {
        const tf2::Vector3 input(x, y, z);
        const tf2::Vector3 output = rotation * input +
          tf2::Vector3(translation.x, translation.y, translation.z);
        points.push_back(PointXYZ{output.x(), output.y(), output.z()});
      } else {
        points.push_back(PointXYZ{x, y, z});
      }
    }
    return points;
  }

  std::vector<PointXYZ> applyPointFilters(const std::vector<PointXYZ> & input) const
  {
    std::vector<PointXYZ> filtered;
    filtered.reserve(input.size());

    for (const auto & point : input) {
      if (point.z < filter_config_.z_min_m || point.z > filter_config_.z_max_m) {
        continue;
      }
      if (self_mask_.isMasked(point)) {
        continue;
      }

      const auto distance_m = range2d(point);
      if (distance_m > filter_config_.max_range_m) {
        continue;
      }

      PointXYZ accepted = point;
      if (distance_m < filter_config_.min_range_m) {
        if (distance_m > 1.0e-6) {
          const double scale = filter_config_.min_range_m / distance_m;
          accepted.x *= scale;
          accepted.y *= scale;
        } else {
          // 原点没有可靠方向，保守映射到机头最近距离，不能作为净空。
          accepted.x = filter_config_.min_range_m;
          accepted.y = 0.0;
        }
      }
      filtered.push_back(accepted);
    }

    if (filter_config_.voxel_enable) {
      filtered = voxelDownsample(filtered);
    }

    if (filter_config_.radius_outlier_enable) {
      filtered = radiusOutlierFilter(filtered);
    }

    return filtered;
  }

  std::vector<PointXYZ> voxelDownsample(const std::vector<PointXYZ> & points) const
  {
    const double leaf = std::max(0.005, filter_config_.voxel_leaf_size_m);
    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;
    voxels.reserve(points.size());

    for (const auto & point : points) {
      const VoxelKey key = gridKeyForPoint(point, leaf);
      auto & accum = voxels[key];
      accum.sum.x += point.x;
      accum.sum.y += point.y;
      accum.sum.z += point.z;
      accum.count += 1;
    }

    std::vector<PointXYZ> output;
    output.reserve(voxels.size());
    for (const auto & item : voxels) {
      const auto & accum = item.second;
      if (accum.count > 0) {
        const double inv = 1.0 / static_cast<double>(accum.count);
        output.push_back(PointXYZ{
          accum.sum.x * inv,
          accum.sum.y * inv,
          accum.sum.z * inv});
      }
    }
    return output;
  }

  std::vector<PointXYZ> radiusOutlierFilter(const std::vector<PointXYZ> & points) const
  {
    if (points.empty()) {
      return {};
    }

    if (filter_config_.radius_outlier_min_neighbors <= 0) {
      return points;
    }

    // 用体素哈希表划分空间邻域，避免对 Livox 的高密度点云执行全量两两比较。
    // 每个点只检查本体素及相邻 26 个体素，计算量随点数近似线性增长。
    const double radius = std::max(0.005, filter_config_.radius_outlier_radius_m);
    const double radius_sq = radius * radius;
    const int min_neighbors = filter_config_.radius_outlier_min_neighbors;

    std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> grid;
    grid.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      grid[gridKeyForPoint(points[i], radius)].push_back(i);
    }

    std::vector<PointXYZ> output;
    output.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto base_key = gridKeyForPoint(points[i], radius);
      int neighbors = 0;

      for (int dx_cell = -1; dx_cell <= 1 && neighbors < min_neighbors; ++dx_cell) {
        for (int dy_cell = -1; dy_cell <= 1 && neighbors < min_neighbors; ++dy_cell) {
          for (int dz_cell = -1; dz_cell <= 1 && neighbors < min_neighbors; ++dz_cell) {
            const VoxelKey neighbor_key{
              base_key.x + dx_cell,
              base_key.y + dy_cell,
              base_key.z + dz_cell};
            const auto bucket = grid.find(neighbor_key);
            if (bucket == grid.end()) {
              continue;
            }

            for (const auto j : bucket->second) {
              if (i == j) {
                continue;
              }
              const double dx = points[i].x - points[j].x;
              const double dy = points[i].y - points[j].y;
              const double dz = points[i].z - points[j].z;
              if (dx * dx + dy * dy + dz * dz <= radius_sq) {
                neighbors += 1;
                if (neighbors >= min_neighbors) {
                  break;
                }
              }
            }
          }
        }
      }

      if (neighbors >= min_neighbors) {
        output.push_back(points[i]);
      }
    }

    return output;
  }

  void publishFilteredCloud(
    const std::vector<PointXYZ> & points,
    const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = filter_config_.target_frame;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & point : points) {
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    filtered_cloud_pub_->publish(cloud);
  }

  void publishMarkers(const SectorArray & distances, const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = filter_config_.target_frame;
    clear.ns = "aero_halo_360";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker rays;
    rays.header.stamp = stamp;
    rays.header.frame_id = filter_config_.target_frame;
    rays.ns = "aero_halo_360";
    rays.id = 1;
    rays.type = visualization_msgs::msg::Marker::LINE_LIST;
    rays.action = visualization_msgs::msg::Marker::ADD;
    rays.scale.x = 0.03;
    rays.color.r = 1.0;
    rays.color.g = 0.1;
    rays.color.b = 0.05;
    rays.color.a = 0.85;

    for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
      const auto distance_cm = distances[sector];
      if (!isObstacleDistance(
          distance_cm,
          temporal_config_.min_distance_cm,
          temporal_config_.max_distance_cm))
      {
        continue;
      }

      const double theta = static_cast<double>(sector) * sectorizer_config_.sector_deg * kPi / 180.0;
      const double distance_m = static_cast<double>(distance_cm) / 100.0;

      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.0;

      geometry_msgs::msg::Point end;
      end.x = std::cos(theta) * distance_m;
      end.y = -std::sin(theta) * distance_m;
      end.z = 0.0;

      rays.points.push_back(origin);
      rays.points.push_back(end);
    }

    array.markers.push_back(rays);
    marker_pub_->publish(array);
  }

  void updateProcessingRate(
    const steady_clock::time_point start,
    const steady_clock::time_point finish)
  {
    (void)finish;
    if (last_process_start_.time_since_epoch().count() != 0) {
      const double dt_s =
        static_cast<double>(duration_cast<milliseconds>(start - last_process_start_).count()) /
        1000.0;
      if (dt_s > 0.0) {
        latest_processing_rate_hz_ = 1.0 / dt_s;
        latest_cloud_interval_ms_ = dt_s * 1000.0;
        cloud_interval_samples_ms_.push_back(latest_cloud_interval_ms_);
        if (cloud_interval_samples_ms_.size() > 200) {
          cloud_interval_samples_ms_.erase(cloud_interval_samples_ms_.begin());
        }
        latest_cloud_interval_max_ms_ = *std::max_element(
          cloud_interval_samples_ms_.begin(), cloud_interval_samples_ms_.end());
        auto sorted_intervals = cloud_interval_samples_ms_;
        std::sort(sorted_intervals.begin(), sorted_intervals.end());
        const auto p95_index = static_cast<std::size_t>(
          std::floor(0.95 * static_cast<double>(sorted_intervals.size() - 1)));
        latest_cloud_interval_p95_ms_ = sorted_intervals[p95_index];
        watchdog_->setProcessingRateHz(latest_processing_rate_hz_);
      }
    }
    latest_processing_latency_ms_ =
      static_cast<double>(duration_cast<std::chrono::microseconds>(finish - start).count()) /
      1000.0;
    last_process_start_ = start;
  }

  void onPublishTimer()
  {
    const auto now = steady_clock::now();
    const auto watchdog_result = watchdog_->evaluate(now);

    SectorArray distances;
    std_msgs::msg::Header header;
    std::size_t filtered_points = 0;
    double processing_rate_hz = 0.0;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      distances = watchdog_result.degraded ? watchdog_->degradedDistances() : latest_distances_;
      header = latest_header_;
      filtered_points = latest_filtered_points_;
      processing_rate_hz = latest_processing_rate_hz_;
    }

    const auto publish_stamp = this->now();
    header.frame_id = filter_config_.target_frame;

    aero_halo_360::msg::SectorDistances msg;
    msg.header = header;
    msg.source_sequence = source_sequence_;
    msg.source_age_ms = static_cast<float>(sourceAgeMsAtPublish(publish_stamp, header));
    std::copy(distances.begin(), distances.end(), msg.distances.begin());
    msg.min_distance_cm = temporal_config_.min_distance_cm;
    msg.max_distance_cm = temporal_config_.max_distance_cm;
    msg.increment_deg = static_cast<float>(sectorizer_config_.sector_deg);
    msg.angle_offset_deg = 0.0F;
    msg.status = static_cast<uint8_t>(watchdog_result.status);
    msg.status_text = watchdog_result.text;
    sector_pub_->publish(msg);

    if (publish_diagnostics_) {
      auto diagnostics_header = header;
      diagnostics_header.stamp = publish_stamp;
      DiagnosticsMetrics metrics;
      metrics.input_topic = filter_config_.input_topic;
      metrics.input_frame = latest_input_frame_;
      metrics.target_frame = filter_config_.target_frame;
      metrics.input_publisher_count = count_publishers(filter_config_.input_topic);
      metrics.configured_publish_rate_hz = filter_config_.publish_rate_hz;
      metrics.profile_name = profile_name_;
      metrics.safety_summary =
        "degraded_mode=" + degraded_mode_name_ +
        ", wall_cm=" + std::to_string(watchdog_config_.virtual_wall_cm) +
        ", warn_ms=" + std::to_string(watchdog_config_.cloud_warn_timeout_ms) +
        ", fail_ms=" + std::to_string(watchdog_config_.cloud_fail_timeout_ms);
      metrics.last_tf_error = last_tf_error_;
      metrics.consecutive_tf_failures = watchdog_result.consecutive_tf_failures;
      metrics.previous_status = healthStatusText(watchdog_result.previous_status);
      metrics.cloud_interval_recent_ms = latest_cloud_interval_ms_;
      metrics.cloud_interval_max_ms = latest_cloud_interval_max_ms_;
      metrics.cloud_interval_p95_ms = latest_cloud_interval_p95_ms_;
      metrics.processing_latency_ms = latest_processing_latency_ms_;
      metrics.source_to_publish_ms = msg.source_age_ms;
      metrics.rejected_pointcloud_count = rejected_pointcloud_count_;
      metrics.last_pointcloud_reject_reason = last_pointcloud_reject_reason_;
      metrics.transition_count = watchdog_result.transition_count;
      metrics.recovery_healthy_frames = watchdog_result.recovery_healthy_frames;
      metrics.transition_reason = watchdog_result.transition_reason;
      diagnostics_pub_->publish(makeDiagnostics(
        diagnostics_header,
        watchdog_result.status,
        watchdog_result.text,
        watchdog_result.cloud_age_ms,
        processing_rate_hz,
        filtered_points,
        watchdog_result.degraded,
        metrics));
    }
  }

  void validateParameters() const
  {
    std::vector<std::string> errors;
    const auto require = [&errors](const bool condition, const std::string & message) {
        if (!condition) {
          errors.push_back(message);
        }
      };

    require(!filter_config_.input_topic.empty(), "input_topic 不能为空");
    require(!filter_config_.target_frame.empty(), "target_frame 不能为空");
    require(
      filter_config_.min_range_m > 0.0 &&
      filter_config_.max_range_m > filter_config_.min_range_m,
      "range_filter 必须满足 0 < min_range_m < max_range_m");
    require(
      filter_config_.max_range_m * 100.0 <= 65534.0,
      "range_filter.max_range_m 不能超过 655.34 m");
    require(
      filter_config_.z_min_m < filter_config_.z_max_m,
      "height_filter 必须满足 z_min_m < z_max_m");
    require(
      !filter_config_.voxel_enable || filter_config_.voxel_leaf_size_m > 0.0,
      "voxel_filter.leaf_size_m 必须大于 0");
    require(
      !filter_config_.radius_outlier_enable ||
      filter_config_.radius_outlier_radius_m > 0.0,
      "radius_outlier.radius_m 必须大于 0");
    require(
      filter_config_.radius_outlier_min_neighbors >= 0,
      "radius_outlier.min_neighbors 不能为负数");
    require(
      std::isfinite(filter_config_.publish_rate_hz) &&
      filter_config_.publish_rate_hz > 0.0,
      "publish_rate_hz 必须是有限正数");
    require(
      filter_config_.timestamp_mode == "live" ||
      filter_config_.timestamp_mode == "replay" ||
      filter_config_.timestamp_mode == "sim",
      "timestamp.mode 只能是 live、replay 或 sim");
    require(
      filter_config_.max_sensor_age_ms > 0 &&
      filter_config_.future_tolerance_ms >= 0 &&
      filter_config_.max_repeated_frames >= 1,
      "timestamp 年龄、未来容差和重复帧阈值必须为有效非负值");
    require(
      filter_config_.max_points > 0 && filter_config_.max_bytes > 0,
      "pointcloud_limits.max_points/max_bytes 必须大于 0");
    require(
      filter_config_.max_nonfinite_fraction >= 0.0 &&
      filter_config_.max_nonfinite_fraction <= 1.0,
      "pointcloud_limits.max_nonfinite_fraction 必须位于 0..1");

    require(
      std::abs(sectorizer_config_.sector_deg - 5.0) < 1.0e-9,
      "sectorizer.sector_deg 固定要求 5.0");
    require(
      sectorizer_config_.radial_bin_width_m > 0.0,
      "sectorizer.radial_bin_width_m 必须大于 0");
    require(
      sectorizer_config_.percentile >= 0.0 &&
      sectorizer_config_.percentile <= 1.0,
      "sectorizer.percentile 必须位于 0..1");
    require(
      sectorizer_config_.near_range_m > 0.0 &&
      sectorizer_config_.mid_range_m >= sectorizer_config_.near_range_m &&
      sectorizer_config_.mid_range_m <= sectorizer_config_.max_range_m,
      "sectorizer 距离层级必须满足 0 < near <= mid <= max");
    require(
      sectorizer_config_.near_min_points >= 1 &&
      sectorizer_config_.mid_min_points >= 1 &&
      sectorizer_config_.far_min_points >= 1,
      "sectorizer 各距离层最小点数必须至少为 1");

    require(temporal_config_.clear_frames >= 1, "temporal_filter.clear_frames 必须至少为 1");
    require(
      temporal_config_.receding_alpha > 0.0 &&
      temporal_config_.receding_alpha <= 1.0,
      "temporal_filter.receding_alpha 必须位于 (0, 1]");
    require(
      inflation_config_.vehicle_radius_m >= 0.0 &&
      inflation_config_.safety_extra_m >= 0.0,
      "vehicle 半径与安全余量不能为负数");
    require(
      inflation_config_.max_inflate_bins >= 0 &&
      inflation_config_.max_inflate_bins <= static_cast<int>(kSectorCount / 2U),
      "inflation.max_inflate_bins 必须位于 0..36");

    if (inflation_config_.enable) {
      const double minimum_distance_m =
        static_cast<double>(inflation_config_.min_distance_cm) / 100.0;
      const int required_bins = static_cast<int>(std::ceil(
        std::atan(
          (inflation_config_.vehicle_radius_m + inflation_config_.safety_extra_m) /
          minimum_distance_m) * 180.0 / kPi / inflation_config_.sector_deg));
      require(
        inflation_config_.max_inflate_bins == 0 ||
        inflation_config_.max_inflate_bins >= required_bins,
        "inflation.max_inflate_bins 小于最近距离所需几何膨胀扇区数 " +
        std::to_string(required_bins));
    }

    require(
      watchdog_config_.cloud_warn_timeout_ms > 0 &&
      watchdog_config_.cloud_fail_timeout_ms > watchdog_config_.cloud_warn_timeout_ms,
      "watchdog 必须满足 0 < cloud_warn_timeout_ms < cloud_fail_timeout_ms");
    require(
      watchdog_config_.recovery_healthy_frames >= 1,
      "watchdog.recovery_healthy_frames 必须至少为 1");
    require(
      watchdog_config_.virtual_wall_cm >= temporal_config_.min_distance_cm &&
      watchdog_config_.virtual_wall_cm <= temporal_config_.max_distance_cm,
      "watchdog.virtual_wall_cm 必须位于 min/max 距离范围内");
    require(
      watchdog_config_.min_processing_rate_hz >= 0.0,
      "watchdog.min_processing_rate_hz 不能为负数");
    require(
      watchdog_config_.tf_failure_limit >= 1 &&
      watchdog_config_.raw_cloud_empty_limit >= 1 &&
      watchdog_config_.empty_cloud_limit >= 1,
      "watchdog 连续失败阈值必须至少为 1");
    require(
      watchdog_config_.filtered_empty_is_clear ||
      !watchdog_config_.filtered_empty_warn_only,
      "filtered_empty_is_clear=false 时 filtered_empty_warn_only 也必须为 false");

    if (!errors.empty()) {
      std::ostringstream stream;
      stream << "AeroHalo360 参数校验失败:";
      for (const auto & error : errors) {
        stream << "\n- " << error;
      }
      throw std::invalid_argument(stream.str());
    }
  }

  bool validateSourceTimestamp(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const int64_t stamp_ns =
      static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(msg.header.stamp.nanosec);
    const int64_t now_ns = this->now().nanoseconds();
    const auto result = source_timestamp_guard_->evaluate(stamp_ns, now_ns);
    latest_source_age_ms_ = result.age_ms;
    source_sequence_ = result.sequence;

    if (!result.accepted) {
      if (result.status != HealthStatus::OK) {
        watchdog_->markSourceInvalid(result.status);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "拒绝点云源时间戳: %s", healthStatusText(result.status).c_str());
      }
      return false;
    }

    watchdog_->markSourceValid();
    return true;
  }
  double sourceAgeMsAtPublish(
    const rclcpp::Time & publish_stamp,
    const std_msgs::msg::Header & source_header) const
  {
    if (source_header.stamp.sec == 0 && source_header.stamp.nanosec == 0) {
      return static_cast<double>(filter_config_.max_sensor_age_ms);
    }
    if (filter_config_.timestamp_mode != "live") {
      return latest_source_age_ms_;
    }
    const rclcpp::Time source_stamp(
      source_header.stamp, get_clock()->get_clock_type());
    return std::max(0.0, (publish_stamp - source_stamp).seconds() * 1000.0);
  }

  CloudFilterConfig filter_config_;
  SectorizerConfig sectorizer_config_;
  TemporalFilterConfig temporal_config_;
  InflationConfig inflation_config_;
  WatchdogConfig watchdog_config_;
  SelfMask self_mask_;

  std::string filtered_cloud_topic_;
  std::string sector_topic_;
  std::string marker_topic_;
  std::string diagnostics_topic_;
  std::string profile_name_{"default"};
  std::string degraded_mode_name_{"virtual_wall"};
  std::string latest_input_frame_{"unknown"};
  std::string last_tf_error_{"NONE"};
  int consecutive_tf_failures_{0};
  bool publish_filtered_cloud_{true};
  bool publish_markers_{true};
  bool publish_diagnostics_{true};

  std::unique_ptr<Sectorizer> sectorizer_;
  std::unique_ptr<TemporalFilter> temporal_filter_;
  std::unique_ptr<Inflation> inflation_;
  std::unique_ptr<Watchdog> watchdog_;
  std::unique_ptr<SourceTimestampGuard> source_timestamp_guard_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<aero_halo_360::msg::SectorDistances>::SharedPtr sector_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex latest_mutex_;
  SectorArray latest_distances_;
  std_msgs::msg::Header latest_header_;
  std::size_t latest_filtered_points_{0};
  double latest_processing_rate_hz_{0.0};
  double latest_cloud_interval_ms_{0.0};
  double latest_cloud_interval_max_ms_{0.0};
  double latest_cloud_interval_p95_ms_{0.0};
  double latest_processing_latency_ms_{0.0};
  std::vector<double> cloud_interval_samples_ms_;
  steady_clock::time_point last_process_start_;
  uint64_t source_sequence_{0};
  double latest_source_age_ms_{0.0};
  uint64_t rejected_pointcloud_count_{0};
  std::string last_pointcloud_reject_reason_{"NONE"};
};

}  // namespace aero_halo_360

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<aero_halo_360::CloudProcessorNode>(options));
  rclcpp::shutdown();
  return 0;
}

