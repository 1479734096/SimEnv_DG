// 参赛方定位节点：只读取初始接口文档授权的 Livox、RGB-D、机身 IMU 和
// 自身 /cmd_vel 输出。正式 adaptive_shared_map 让 RGB-D 与 Livox 对同一
// 局部地图生成平移候选，IMU 固定姿态，只有最终选中的位姿和当前 Livox 帧可事务提交。
#include <algorithm>
#include <array>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <danger_search_localization/point_range_filter.hpp>
#include <danger_search_localization/adaptive_translation_selector.hpp>
#include <danger_search_localization/ground_plane_initializer.hpp>
#include <danger_search_localization/imu_constrained_odometry.hpp>
#include <danger_search_localization/imu_motion_predictor.hpp>
#include <danger_search_localization/planar_gicp_odometry.hpp>
#include <danger_search_localization/ros_time_window.hpp>
#include <danger_search_localization/pose_velocity_estimator.hpp>
#include <danger_search_localization/stance_health_monitor.hpp>
#include <danger_search_localization/stance_initialization_gate.hpp>
#include <danger_search_localization/velocity_transform.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>

#include "kiss_icp/pipeline/KissICP.hpp"

namespace {

using Sophus::SE3d;
using Sophus::SO3d;

double YawFromRotation(const Eigen::Matrix3d &rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

SO3d RotationFromRpy(const std::vector<double> &rpy) {
  if (rpy.size() != 3) {
    throw std::runtime_error("base_to_livox_rpy 必须包含 3 个值");
  }
  if (!std::all_of(rpy.begin(), rpy.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("传感器固定外参 rpy 包含非有限数值");
  }
  const Eigen::AngleAxisd roll(rpy[0], Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy[1], Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy[2], Eigen::Vector3d::UnitZ());
  return SO3d(yaw * pitch * roll);
}

Eigen::Vector3d VectorFromParam(const std::vector<double> &values, const std::string &name) {
  if (values.size() != 3) {
    throw std::runtime_error(name + " 必须包含 3 个值");
  }
  if (!std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error(name + " 包含非有限数值");
  }
  return {values[0], values[1], values[2]};
}

struct StartPose {
  // x/y/yaw 来自公开 team_scene_info；z 只保留用于诊断，正式世界系高度由
  // 首帧楼面与 base 离地距离初始化。
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  bool relative = false;
};

struct PlanarRegistrationDiagnosticStats {
  std::array<
      std::uint64_t,
      static_cast<std::size_t>(
          danger_search_localization::PlanarGicpStatus::kCount)>
      status_counts{};
  ros::Time interval_start;
  double maximum_candidate_norm = 0.0;
  double maximum_candidate_rotation_angle = 0.0;
};

struct MapRegistrationDiagnosticStats {
  std::array<
      std::uint64_t,
      static_cast<std::size_t>(
          danger_search_localization::ImuConstrainedOdometryStatus::kCount)>
      status_counts{};
  ros::Time interval_start;
  double maximum_unclamped_norm = 0.0;
  double maximum_applied_norm = 0.0;
  std::uint64_t clamped_count = 0;
};

struct MotionConsistencyDiagnosticState {
  bool initialized = false;
  ros::Time interval_start;
  Eigen::Vector2d depth_anchor = Eigen::Vector2d::Zero();
  Eigen::Vector2d livox_anchor = Eigen::Vector2d::Zero();
  Eigen::Vector2d map_correction_anchor = Eigen::Vector2d::Zero();
  Eigen::Vector2d fused_anchor = Eigen::Vector2d::Zero();
  Eigen::Vector2d command_anchor = Eigen::Vector2d::Zero();
  double previous_yaw = 0.0;
  double absolute_yaw_change = 0.0;
  std::size_t frame_count = 0;
  std::size_t depth_fresh_count = 0;
};

std::filesystem::path ExpectedSceneInfoPath() {
  const std::string package = ros::package::getPath("danger_search");
  if (package.empty()) {
    throw std::runtime_error("无法定位 danger_search ROS 包");
  }
  const auto package_path = std::filesystem::absolute(package).lexically_normal();
  return (package_path.parent_path().parent_path() / "generated_building" /
          "team_scene_info.json").lexically_normal();
}

std::filesystem::path ValidateSceneInfoPath(const std::string &raw) {
  if (!raw.empty() && raw.front() == '~') {
    throw std::runtime_error("正式定位场景文件路径不支持 shell 缩写");
  }
  const auto supplied = std::filesystem::absolute(raw).lexically_normal();
  const auto expected = std::filesystem::absolute(ExpectedSceneInfoPath()).lexically_normal();
  if (supplied != expected) {
    throw std::runtime_error("正式定位场景文件路径不合法: " + supplied.string());
  }
  if (std::filesystem::is_symlink(supplied.parent_path()) ||
      std::filesystem::is_symlink(supplied)) {
    throw std::runtime_error("公开场景文件及其目录不能是符号链接: " + supplied.string());
  }
  if (!std::filesystem::is_regular_file(supplied)) {
    throw std::runtime_error("公开场景文件必须是真实普通文件: " + supplied.string());
  }
  const auto supplied_canonical = std::filesystem::canonical(supplied);
  const auto expected_canonical = std::filesystem::canonical(expected);
  if (supplied_canonical != expected_canonical) {
    throw std::runtime_error("公开场景文件规范路径不一致: " + supplied_canonical.string());
  }
  return supplied_canonical;
}

StartPose ReadPublicStartPose(const std::string &path, bool require_info,
                              bool allow_relative, bool require_trunk_imu,
                              bool require_depth_points,
                              bool require_command_topic) {
  // C++ 定位节点独立复核 Python 主节点的文件身份、schema 和所需接口，避免
  // launch 只校验一侧后由另一侧形成未授权输入旁路。
  boost::property_tree::ptree tree;
  try {
    const auto validated_path = ValidateSceneInfoPath(path);
    boost::property_tree::read_json(validated_path.string(), tree);
    if (tree.get<std::string>("schema") != "team_scene_info_v1") {
      throw std::runtime_error("team_scene_info schema 不是 team_scene_info_v1");
    }
    if (tree.get<std::string>("coordinate_frame") != "world") {
      throw std::runtime_error("team_scene_info 坐标系必须是 world");
    }
    StartPose pose;
    pose.x = tree.get<double>("robot_start.x");
    pose.y = tree.get<double>("robot_start.y");
    pose.z = tree.get<double>("robot_start.z");
    pose.yaw = tree.get<double>("robot_start.yaw");
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.z) || !std::isfinite(pose.yaw)) {
      throw std::runtime_error("team_scene_info robot_start 包含非有限数值");
    }
    bool scan_allowed = false;
    bool trunk_imu_allowed = false;
    bool depth_points_allowed = false;
    bool command_topic_allowed = false;
    for (const auto &entry : tree.get_child("allowed_interfaces.topics")) {
      const std::string topic = entry.second.get_value<std::string>();
      if (topic == "/scan") {
        scan_allowed = true;
      } else if (topic == "/trunk_imu") {
        trunk_imu_allowed = true;
      } else if (topic == "/real_sense/depth/points") {
        depth_points_allowed = true;
      } else if (topic == "/cmd_vel") {
        command_topic_allowed = true;
      }
    }
    if (!scan_allowed) {
      throw std::runtime_error("公开场景信息未授权 /scan");
    }
    if (require_trunk_imu && !trunk_imu_allowed) {
      throw std::runtime_error("公开场景信息未授权 /trunk_imu");
    }
    if (require_depth_points && !depth_points_allowed) {
      throw std::runtime_error("公开场景信息未授权 /real_sense/depth/points");
    }
    if (require_command_topic && !command_topic_allowed) {
      throw std::runtime_error("公开场景信息未授权 /cmd_vel");
    }
    return pose;
  } catch (const std::exception &error) {
    if (require_info || !allow_relative) {
      throw std::runtime_error("无法读取公开 team_scene_info.json: " + std::string(error.what()));
    }
    ROS_WARN("未读取 team_scene_info.json，诊断模式使用相对原点：%s", error.what());
    StartPose pose;
    pose.relative = true;
    return pose;
  }
}

class ScanLocalizationNode {
 public:
  /// 校验全部参数和公开输入后建立订阅/发布关系。
  ///
  /// 支持五个固定后端；正式 launch 默认选择 adaptive_shared_map。构造期间任何接口、外参或场景
  /// 信息异常都会抛出并终止节点，不会退回 Gazebo 真值或相对原点。
  ScanLocalizationNode() : nh_(), private_nh_("~") {
    std::string raw_cloud_topic = "/scan";
    std::string localization_backend = "kiss_icp";
    std::string scene_info_path;
    bool require_scene_info = true;
    bool allow_relative_origin = false;
    double min_range = 0.5;
    double max_range = 15.0;
    double voxel_size = 0.15;
    double imu_max_delta = 0.01;
    double command_velocity_scale = 1.0;
    double initialization_min_base_height = 0.20;
    double initialization_max_base_height = 0.55;
    double initialization_max_height_span = 0.04;
    double initialization_max_angular_speed = 0.30;
    int initialization_stable_frames = 8;
    double health_min_base_height = 0.14;
    double health_max_base_height = 0.60;
    double health_min_ground_normal_z = 0.85;
    double health_min_base_up_z = 0.71;
    double health_max_angular_speed = 1.50;
    int health_required_healthy_frames = 8;
    int health_required_unhealthy_frames = 3;
    int health_required_unknown_frames = 30;
    bool health_latch_unhealthy = true;
    bool stance_health_enabled = true;
    double velocity_estimation_window = 1.00;
    double velocity_minimum_interval = 0.80;
    double velocity_linear_deadband = 0.05;
    double velocity_angular_deadband = 0.03;
    double diagnostic_large_translation_threshold = 0.08;
    double diagnostic_summary_period = 2.0;
    double scan_to_map_maximum_correction = 0.10;
    double livox_imu_maximum_speed = 0.70;
    double livox_imu_translation_margin = 0.02;
    double livox_imu_absolute_translation_cap = 0.30;
    double shared_map_depth_max_age = 0.35;
    double shared_map_max_source_divergence = 0.05;
    double shared_map_max_map_commit_difference = 0.05;
    bool diagnostic_frame_details = false;
    std::string depth_cloud_topic = "/real_sense/depth/points";
    std::string raw_cloud_frame = "laser_livox";
    std::string imu_frame = "imu_link";
    std::string depth_cloud_frame = "real_sense_optical_frame";
    std::vector<double> base_to_livox_xyz;
    std::vector<double> base_to_livox_rpy;
    std::vector<double> base_to_depth_xyz;
    std::vector<double> base_to_depth_rpy;

    private_nh_.param("raw_cloud_topic", raw_cloud_topic, raw_cloud_topic);
    private_nh_.param("localization_backend", localization_backend,
                      localization_backend);
    private_nh_.param("team_scene_info_path", scene_info_path, std::string());
    private_nh_.param("require_team_scene_info", require_scene_info, require_scene_info);
    private_nh_.param("allow_relative_origin", allow_relative_origin, allow_relative_origin);
    private_nh_.param("min_range", min_range, min_range);
    private_nh_.param("max_range", max_range, max_range);
    private_nh_.param("voxel_size", voxel_size, voxel_size);
    private_nh_.param("imu_max_delta", imu_max_delta, imu_max_delta);
    private_nh_.param("command_velocity_scale", command_velocity_scale,
                      command_velocity_scale);
    private_nh_.param("initialization_min_base_height",
                      initialization_min_base_height,
                      initialization_min_base_height);
    private_nh_.param("initialization_max_base_height",
                      initialization_max_base_height,
                      initialization_max_base_height);
    private_nh_.param("initialization_max_height_span",
                      initialization_max_height_span,
                      initialization_max_height_span);
    private_nh_.param("initialization_max_angular_speed",
                      initialization_max_angular_speed,
                      initialization_max_angular_speed);
    private_nh_.param("initialization_stable_frames",
                      initialization_stable_frames,
                      initialization_stable_frames);
    private_nh_.param("health_min_base_height", health_min_base_height,
                      health_min_base_height);
    private_nh_.param("health_max_base_height", health_max_base_height,
                      health_max_base_height);
    private_nh_.param("health_min_ground_normal_z", health_min_ground_normal_z,
                      health_min_ground_normal_z);
    private_nh_.param("health_min_base_up_z", health_min_base_up_z,
                      health_min_base_up_z);
    private_nh_.param("health_max_angular_speed", health_max_angular_speed,
                      health_max_angular_speed);
    private_nh_.param("health_required_healthy_frames",
                      health_required_healthy_frames,
                      health_required_healthy_frames);
    private_nh_.param("health_required_unhealthy_frames",
                      health_required_unhealthy_frames,
                      health_required_unhealthy_frames);
    private_nh_.param("health_required_unknown_frames",
                      health_required_unknown_frames,
                      health_required_unknown_frames);
    private_nh_.param("health_latch_unhealthy", health_latch_unhealthy,
                      health_latch_unhealthy);
    private_nh_.param("stance_health_enabled", stance_health_enabled,
                      stance_health_enabled);
    private_nh_.param("velocity_estimation_window", velocity_estimation_window,
                      velocity_estimation_window);
    private_nh_.param("velocity_minimum_interval", velocity_minimum_interval,
                      velocity_minimum_interval);
    private_nh_.param("velocity_linear_deadband", velocity_linear_deadband,
                      velocity_linear_deadband);
    private_nh_.param("velocity_angular_deadband", velocity_angular_deadband,
                      velocity_angular_deadband);
    private_nh_.param("diagnostic_large_translation_threshold",
                      diagnostic_large_translation_threshold,
                      diagnostic_large_translation_threshold);
    private_nh_.param("diagnostic_summary_period", diagnostic_summary_period,
                      diagnostic_summary_period);
    private_nh_.param("scan_to_map_maximum_correction",
                      scan_to_map_maximum_correction,
                      scan_to_map_maximum_correction);
    private_nh_.param("livox_imu_maximum_speed", livox_imu_maximum_speed,
                      livox_imu_maximum_speed);
    private_nh_.param("livox_imu_translation_margin",
                      livox_imu_translation_margin,
                      livox_imu_translation_margin);
    private_nh_.param("livox_imu_absolute_translation_cap",
                      livox_imu_absolute_translation_cap,
                      livox_imu_absolute_translation_cap);
    private_nh_.param("shared_map_depth_max_age", shared_map_depth_max_age,
                      shared_map_depth_max_age);
    private_nh_.param("shared_map_max_source_divergence",
                      shared_map_max_source_divergence,
                      shared_map_max_source_divergence);
    private_nh_.param("shared_map_max_map_commit_difference",
                      shared_map_max_map_commit_difference,
                      shared_map_max_map_commit_difference);
    private_nh_.param("diagnostic_frame_details", diagnostic_frame_details,
                      diagnostic_frame_details);
    private_nh_.param("depth_cloud_topic", depth_cloud_topic,
                      depth_cloud_topic);
    private_nh_.param("raw_cloud_frame", raw_cloud_frame, raw_cloud_frame);
    private_nh_.param("imu_frame", imu_frame, imu_frame);
    private_nh_.param("depth_cloud_frame", depth_cloud_frame,
                      depth_cloud_frame);
    if (raw_cloud_frame.empty() || imu_frame.empty() ||
        depth_cloud_frame.empty()) {
      throw std::runtime_error("定位输入 frame_id 契约不能为空");
    }
    raw_cloud_frame_ = raw_cloud_frame;
    imu_frame_ = imu_frame;
    depth_cloud_frame_ = depth_cloud_frame;
    if (!std::isfinite(min_range) || !std::isfinite(max_range) || min_range < 0.0 ||
        max_range < min_range) {
      throw std::runtime_error("定位点云量程参数无效");
    }
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0 ||
        !std::isfinite(imu_max_delta) || imu_max_delta <= 0.0 ||
        !std::isfinite(command_velocity_scale) ||
        command_velocity_scale <= 0.0 || command_velocity_scale > 2.0) {
      throw std::runtime_error("KISS-ICP 定位参数无效");
    }
    if (!std::isfinite(diagnostic_large_translation_threshold) ||
        diagnostic_large_translation_threshold <= 0.0 ||
        diagnostic_large_translation_threshold > 1.0) {
      throw std::runtime_error("定位大平移诊断阈值必须位于 (0, 1.0] m");
    }
    if (!std::isfinite(diagnostic_summary_period) ||
        diagnostic_summary_period < 0.5 || diagnostic_summary_period > 30.0) {
      throw std::runtime_error("定位诊断汇总周期必须位于 [0.5, 30.0] s");
    }
    if (!std::isfinite(scan_to_map_maximum_correction) ||
        scan_to_map_maximum_correction <= 0.0 ||
        scan_to_map_maximum_correction > 0.25) {
      throw std::runtime_error("scan-to-map 单帧校正上限必须位于 (0, 0.25] m");
    }
    if (!std::isfinite(livox_imu_maximum_speed) ||
        livox_imu_maximum_speed <= 0.0 ||
        !std::isfinite(livox_imu_translation_margin) ||
        livox_imu_translation_margin < 0.0 ||
        !std::isfinite(livox_imu_absolute_translation_cap) ||
        livox_imu_absolute_translation_cap <= 0.0) {
      throw std::runtime_error("Livox-IMU 平面运动学上界参数无效");
    }
    if (!std::isfinite(shared_map_depth_max_age) ||
        shared_map_depth_max_age <= 0.0 || shared_map_depth_max_age > 1.0 ||
        !std::isfinite(shared_map_max_source_divergence) ||
        shared_map_max_source_divergence <= 0.0 ||
        shared_map_max_source_divergence > 0.30 ||
        !std::isfinite(shared_map_max_map_commit_difference) ||
        shared_map_max_map_commit_difference <= 0.0 ||
        shared_map_max_map_commit_difference > 0.30) {
      throw std::runtime_error(
          "共享地图候选年龄必须位于 (0, 1.0] s，距离阈值必须位于 (0, 0.30] m");
    }
    if (localization_backend != "kiss_icp" &&
        localization_backend != "imu_constrained" &&
        localization_backend != "livox_imu_local_map" &&
        localization_backend != "adaptive_shared_map" &&
        localization_backend != "rgbd_livox") {
      throw std::runtime_error(
          "localization_backend 仅支持 kiss_icp、imu_constrained、"
          "livox_imu_local_map、adaptive_shared_map 或 rgbd_livox");
    }
    if (!private_nh_.getParam("base_to_livox_xyz", base_to_livox_xyz) ||
        !private_nh_.getParam("base_to_livox_rpy", base_to_livox_rpy)) {
      throw std::runtime_error("缺少已公开的 base_to_livox 固定外参");
    }
    if (raw_cloud_topic != "/scan") {
      throw std::runtime_error("正式定位仅允许使用初始文档授权的 /scan");
    }
    use_imu_constrained_ = localization_backend == "imu_constrained";
    use_livox_imu_local_map_ =
        localization_backend == "livox_imu_local_map";
    use_adaptive_shared_map_ =
        localization_backend == "adaptive_shared_map";
    use_rgbd_livox_ = localization_backend == "rgbd_livox";
    const bool requires_imu =
        use_imu_constrained_ || use_livox_imu_local_map_ ||
        use_adaptive_shared_map_ || use_rgbd_livox_ || stance_health_enabled;
    if ((use_rgbd_livox_ || use_adaptive_shared_map_) &&
        depth_cloud_topic != "/real_sense/depth/points") {
      throw std::runtime_error(
          "RGB-D/Livox 定位仅允许使用 /real_sense/depth/points");
    }
    const StartPose start = ReadPublicStartPose(
        scene_info_path, require_scene_info, allow_relative_origin,
        requires_imu, use_rgbd_livox_ || use_adaptive_shared_map_,
        use_livox_imu_local_map_ || use_adaptive_shared_map_ || use_rgbd_livox_);
    if ((use_imu_constrained_ || use_livox_imu_local_map_ ||
         use_adaptive_shared_map_ || use_rgbd_livox_) &&
        start.relative) {
      throw std::runtime_error("IMU 融合定位必须使用公开世界系起点");
    }
    start_pose_ = start;
    base_to_livox_ = SE3d(RotationFromRpy(base_to_livox_rpy),
                           VectorFromParam(base_to_livox_xyz, "base_to_livox_xyz"));
    if (use_rgbd_livox_ || use_adaptive_shared_map_) {
      if (!private_nh_.getParam("base_to_depth_xyz", base_to_depth_xyz) ||
          !private_nh_.getParam("base_to_depth_rpy", base_to_depth_rpy)) {
        throw std::runtime_error("缺少已公开的 base_to_depth 固定外参");
      }
      base_to_depth_ = SE3d(
          RotationFromRpy(base_to_depth_rpy),
          VectorFromParam(base_to_depth_xyz, "base_to_depth_xyz"));
    }
    frame_id_ = start.relative ? "danger_search_relative" : "world";
    if (start.relative) {
      const SE3d world_base0(
          SO3d(Eigen::AngleAxisd(start.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()),
          Eigen::Vector3d(start.x, start.y, start.z));
      world_livox0_ = world_base0 * base_to_livox_;
      world_frame_initialized_ = true;
    }

    kiss_icp::pipeline::KISSConfig config;
    config.min_range = min_range;
    config.max_range = max_range;
    config.voxel_size = voxel_size;
    config.deskew = false;
    odometry_ = kiss_icp::pipeline::KissICP(config);
    min_range_ = min_range;
    max_range_ = max_range;
    imu_max_delta_ = imu_max_delta;
    command_velocity_scale_ = command_velocity_scale;
    livox_imu_maximum_speed_ = livox_imu_maximum_speed;
    livox_imu_translation_margin_ = livox_imu_translation_margin;
    livox_imu_absolute_translation_cap_ =
        livox_imu_absolute_translation_cap;
    adaptive_selector_config_.maximum_proposal_age = shared_map_depth_max_age;
    adaptive_selector_config_.maximum_source_divergence =
        shared_map_max_source_divergence;
    adaptive_selector_config_.maximum_map_commit_difference =
        shared_map_max_map_commit_difference;
    diagnostic_large_translation_threshold_ =
        diagnostic_large_translation_threshold;
    diagnostic_summary_period_ = diagnostic_summary_period;
    diagnostic_frame_details_ = diagnostic_frame_details;
    stance_health_enabled_ = stance_health_enabled;
    danger_search_localization::StanceInitializationConfig stance_config;
    stance_config.minimum_base_height = initialization_min_base_height;
    stance_config.maximum_base_height = initialization_max_base_height;
    stance_config.maximum_height_span = initialization_max_height_span;
    stance_config.maximum_angular_speed = initialization_max_angular_speed;
    stance_config.required_consecutive_frames =
        static_cast<std::size_t>(std::max(0, initialization_stable_frames));
    stance_initialization_gate_ =
        danger_search_localization::StanceInitializationGate(stance_config);
    danger_search_localization::StanceHealthMonitorConfig health_config;
    health_config.minimum_base_height = health_min_base_height;
    health_config.maximum_base_height = health_max_base_height;
    health_config.minimum_ground_normal_z = health_min_ground_normal_z;
    health_config.minimum_base_up_z = health_min_base_up_z;
    health_config.maximum_angular_speed = health_max_angular_speed;
    health_config.required_healthy_frames =
        static_cast<std::size_t>(std::max(0, health_required_healthy_frames));
    health_config.required_unhealthy_frames =
        static_cast<std::size_t>(std::max(0, health_required_unhealthy_frames));
    health_config.required_unknown_frames =
        static_cast<std::size_t>(std::max(0, health_required_unknown_frames));
    health_config.latch_unhealthy = health_latch_unhealthy;
    stance_health_monitor_ =
        danger_search_localization::StanceHealthMonitor(health_config);
    danger_search_localization::PoseVelocityConfig velocity_config;
    velocity_config.window_seconds = velocity_estimation_window;
    velocity_config.minimum_interval = velocity_minimum_interval;
    velocity_config.linear_deadband = velocity_linear_deadband;
    velocity_config.angular_deadband = velocity_angular_deadband;
    velocity_estimator_ =
        danger_search_localization::PoseVelocityEstimator(velocity_config);
    if (use_imu_constrained_ || use_livox_imu_local_map_ ||
        use_adaptive_shared_map_ || use_rgbd_livox_) {
      danger_search_localization::ImuConstrainedOdometryConfig imu_config;
      // 正式后端把 scan-to-map 用作慢漂校正，候选后端用它估计完整平移；
      // 两者都让返回位姿和地图写入使用同一校正，避免形成两个坐标系。
      imu_config.maximum_correction_from_prior =
          scan_to_map_maximum_correction;
      imu_odometry_ = std::make_unique<
          danger_search_localization::ImuConstrainedOdometry>(imu_config);
    }
    if (use_rgbd_livox_) {
      danger_search_localization::PlanarGicpConfig depth_config;
      depth_config.voxel_size = 0.08;
      depth_config.maximum_correspondence_distance = 0.25;
      depth_config.maximum_fitness = 0.04;
      depth_gicp_ =
          danger_search_localization::PlanarGicpOdometry(depth_config);
    }

    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(
        "/danger_search/localization/odom", 4);
    cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/danger_search/localization/cloud_registered", 2);
    ready_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/danger_search/localization/ready", 1, true);
    health_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/danger_search/localization/health", 1, true);
    std_msgs::Bool not_ready;
    not_ready.data = false;
    ready_publisher_.publish(not_ready);
    std_msgs::Bool not_healthy;
    not_healthy.data = false;
    health_publisher_.publish(not_healthy);
    subscriber_ = nh_.subscribe(raw_cloud_topic, 2,
                                &ScanLocalizationNode::OnCloud, this);
    if (requires_imu) {
      imu_subscriber_ = nh_.subscribe(
          "/trunk_imu", 200, &ScanLocalizationNode::OnImu, this);
    }
    if (use_livox_imu_local_map_ || use_adaptive_shared_map_ ||
        use_rgbd_livox_) {
      command_subscriber_ = nh_.subscribe(
          "/cmd_vel", 50, &ScanLocalizationNode::OnCommand, this);
    }
    if (use_rgbd_livox_ || use_adaptive_shared_map_) {
      depth_subscriber_ = nh_.subscribe(
          depth_cloud_topic, 1, &ScanLocalizationNode::OnDepthCloud, this);
    }
    ROS_INFO(
        "scan_localization 已启动：输入 /scan，后端=%s，deskew 关闭，坐标系=%s",
        localization_backend.c_str(), frame_id_.c_str());
  }

 private:
  struct ImuSample {
    ros::Time stamp;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
  };

  struct CommandSample {
    ros::Time stamp;
    Eigen::Vector2d base_velocity;
  };

  void OnCommand(const geometry_msgs::Twist::ConstPtr &message) {
    // 缓存本算法已经发出的机体系平移命令，按时间积分后仅作为配准初值；
    // 点云证据仍决定是否接受位移，因此控制命令不能自证定位结果。
    const double linear_x = message->linear.x;
    const double linear_y = message->linear.y;
    if (!std::isfinite(linear_x) || !std::isfinite(linear_y)) {
      return;
    }
    const ros::Time stamp = ros::Time::now();
    if (stamp.isZero()) {
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_samples_.push_back(
        CommandSample{stamp, Eigen::Vector2d(linear_x, linear_y)});
    const ros::Time oldest =
        danger_search_localization::RetentionWindowStart(stamp, 5.0);
    while (!command_samples_.empty() &&
           command_samples_.front().stamp < oldest) {
      command_samples_.pop_front();
    }
  }

  Eigen::Vector2d CommandMotionPrior(
      const ros::Time &start, const ros::Time &end,
      const Eigen::Matrix3d &world_base_rotation) const {
    // 对 [start,end] 内的分段常值命令积分，并用当前 IMU 姿态旋到 world XY。
    if (start.isZero() || end <= start || !world_base_rotation.allFinite()) {
      return Eigen::Vector2d::Zero();
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    Eigen::Vector2d current_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d result = Eigen::Vector2d::Zero();
    ros::Time cursor = start;
    for (const auto &sample : command_samples_) {
      if (sample.stamp <= start) {
        current_velocity = sample.base_velocity;
        continue;
      }
      if (sample.stamp >= end) {
        break;
      }
      const double dt = sample.stamp.toSec() - cursor.toSec();
      if (dt > 0.0) {
        const Eigen::Vector3d world_velocity =
            world_base_rotation *
            Eigen::Vector3d(current_velocity.x(), current_velocity.y(), 0.0);
        result += world_velocity.head<2>() * dt;
      }
      cursor = sample.stamp;
      current_velocity = sample.base_velocity;
    }
    const double final_dt = end.toSec() - cursor.toSec();
    if (final_dt > 0.0) {
      const Eigen::Vector3d world_velocity =
          world_base_rotation *
          Eigen::Vector3d(current_velocity.x(), current_velocity.y(), 0.0);
      result += world_velocity.head<2>() * final_dt;
    }
    return command_velocity_scale_ * result;
  }

  void OnImu(const sensor_msgs::Imu::ConstPtr &message) {
    // 只缓存有限、归一化且带有效时间戳的 IMU；livox_imu_local_map 与
    // rgbd_livox 在各点云回调中按时间查最近样本，imu_constrained 还会在此
    // 更新短时平移积分先验。
    if (message->header.frame_id != imu_frame_) {
      ROS_ERROR_THROTTLE(
          2.0, "/trunk_imu frame_id 不符合公开契约：实际=%s，期望=%s",
          message->header.frame_id.c_str(), imu_frame_.c_str());
      return;
    }
    const auto &q = message->orientation;
    const auto &a = message->linear_acceleration;
    const auto &w = message->angular_velocity;
    if (message->header.stamp.isZero() || !std::isfinite(q.w) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) {
      return;
    }
    Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
      return;
    }
    orientation.normalize();
    const ImuSample sample{
        message->header.stamp,
        orientation.toRotationMatrix(),
        Eigen::Vector3d(a.x, a.y, a.z),
        Eigen::Vector3d(w.x, w.y, w.z)};
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_samples_.push_back(sample);
      // 深度配准与 Livox 配准共享 CPU；保留两秒 IMU 历史，确保短时计算
      // 抖动不会把时间戳正确的点云误判为无法匹配。
      const ros::Time oldest =
          danger_search_localization::RetentionWindowStart(
              message->header.stamp, 2.0);
      while (!imu_samples_.empty() && imu_samples_.front().stamp < oldest) {
        imu_samples_.pop_front();
      }
    }
    if (use_imu_constrained_) {
      std::lock_guard<std::mutex> lock(processing_mutex_);
      if (!imu_anchor_initialized_) {
        return;
      }
      const Eigen::Matrix3d world_base_rotation =
          danger_search_localization::AnchorImuOrientation(
              initial_world_base_rotation_, initial_imu_rotation_,
              sample.rotation);
      imu_motion_predictor_.AddSample(
          sample.stamp.toSec(), world_base_rotation,
          sample.linear_acceleration, sample.angular_velocity);
    }
  }

  std::optional<ImuSample> ImuSampleAt(
      const ros::Time &stamp, double *closest_delta_out = nullptr) const {
    // 不做跨时间插值：最近样本仍须位于 imu_max_delta_，否则整帧点云跳过。
    std::lock_guard<std::mutex> lock(imu_mutex_);
    const ImuSample *closest = nullptr;
    double closest_delta = std::numeric_limits<double>::infinity();
    for (const auto &sample : imu_samples_) {
      const double delta = std::abs(sample.stamp.toSec() - stamp.toSec());
      if (delta < closest_delta) {
        closest = &sample;
        closest_delta = delta;
      }
    }
    if (closest_delta_out != nullptr) {
      *closest_delta_out = closest_delta;
    }
    if (closest == nullptr || closest_delta > imu_max_delta_) {
      return std::nullopt;
    }
    return *closest;
  }

  static double Median(std::vector<double> values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
  }

  void UpdateLivoxScale(const ros::Time &stamp) {
    // 以约一秒窗口比较 RGB-D 与 Livox 累计里程，使用最近若干合法比值的中位数
    // 修正 Livox 相对位移尺度；异常范围样本不进入历史。
    if (!scale_window_initialized_) {
      scale_depth_anchor_ = depth_total_;
      scale_livox_anchor_ = livox_total_;
      scale_window_stamp_ = stamp;
      scale_window_initialized_ = true;
      return;
    }
    if (stamp.toSec() - scale_window_stamp_.toSec() < 1.0) {
      return;
    }
    const double depth_distance =
        (depth_total_ - scale_depth_anchor_).norm();
    const double livox_distance =
        (livox_total_ - scale_livox_anchor_).norm();
    double ratio = std::numeric_limits<double>::quiet_NaN();
    const char *status = "insufficient_depth_motion";
    const double previous_scale = livox_scale_;
    if (depth_distance >= 0.20 && livox_distance >= 0.10) {
      ratio = depth_distance / livox_distance;
      if (std::isfinite(ratio) && ratio >= 0.70 && ratio <= 2.20) {
        livox_scale_samples_.push_back(ratio);
        while (livox_scale_samples_.size() > 5) {
          livox_scale_samples_.pop_front();
        }
        livox_scale_ = Median(std::vector<double>(
            livox_scale_samples_.begin(), livox_scale_samples_.end()));
        status = "accepted";
      } else if (!std::isfinite(ratio)) {
        status = "nonfinite_ratio";
      } else {
        status = "ratio_out_of_range";
      }
    } else if (depth_distance >= 0.20) {
      status = "insufficient_livox_motion";
    }
    ROS_INFO(
        "localization_scale_window stamp=%.6f status=%s "
        "depth_distance=%.4f livox_distance=%.4f ratio=%.4f "
        "previous_scale=%.4f current_scale=%.4f samples=%zu",
        stamp.toSec(), status, depth_distance, livox_distance, ratio,
        previous_scale, livox_scale_, livox_scale_samples_.size());
    scale_depth_anchor_ = depth_total_;
    scale_livox_anchor_ = livox_total_;
    scale_window_stamp_ = stamp;
  }

  void OnDepthCloud(const sensor_msgs::PointCloud2::ConstPtr &message) {
    // 正式旧后端继续累计帧间 GICP；共享地图候选只读估计绝对平移，不在深度回调写状态。
    if ((!use_rgbd_livox_ && !use_adaptive_shared_map_) ||
        message->header.stamp.isZero()) {
      return;
    }
    if (message->header.frame_id != depth_cloud_frame_) {
      ROS_ERROR_THROTTLE(
          2.0,
          "/real_sense/depth/points frame_id 不符合公开契约：实际=%s，期望=%s",
          message->header.frame_id.c_str(), depth_cloud_frame_.c_str());
      return;
    }
    std::lock_guard<std::mutex> depth_lock(depth_processing_mutex_);
    Eigen::Matrix3d initial_world_base_rotation;
    Eigen::Matrix3d initial_imu_rotation;
    double base_height = 0.0;
    {
      std::lock_guard<std::mutex> state_lock(initialization_mutex_);
      if (!world_frame_initialized_ ||
          (use_rgbd_livox_ && !fusion_initialized_) ||
          (use_adaptive_shared_map_ && !adaptive_pose_initialized_)) {
        return;
      }
      initial_world_base_rotation = initial_world_base_rotation_;
      initial_imu_rotation = initial_imu_rotation_;
      base_height = initial_world_base_translation_.z();
    }
    if (!last_depth_process_stamp_.isZero() &&
        message->header.stamp.toSec() - last_depth_process_stamp_.toSec() < 0.18) {
      return;
    }
    last_depth_process_stamp_ = message->header.stamp;
    double imu_delta = std::numeric_limits<double>::infinity();
    const auto imu_sample = ImuSampleAt(message->header.stamp, &imu_delta);
    if (!imu_sample.has_value()) {
      ROS_WARN_THROTTLE(
          2.0,
          "/trunk_imu 与深度点云时间不匹配，跳过深度定位帧（最近差 %.3f s）",
          imu_delta);
      return;
    }
    const Eigen::Matrix3d world_base_rotation =
        danger_search_localization::AnchorImuOrientation(
            initial_world_base_rotation, initial_imu_rotation,
            imu_sample->rotation);
    const Eigen::Matrix3d world_depth_rotation =
        world_base_rotation * base_to_depth_.rotationMatrix();
    std::vector<Eigen::Vector3d> points;
    points.reserve(message->width * message->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      std::size_t point_index = 0;
      for (; x != x.end(); ++x, ++y, ++z, ++point_index) {
        // 原始深度点云为 640x480；GICP 最终按 8 cm 体素工作，逐像素
        // 输入只增加计算延迟。规则采样保留完整视场和几何边界。
        if (point_index % 4 != 0) {
          continue;
        }
        const Eigen::Vector3d point(*x, *y, *z);
        const double range = point.norm();
        if (!point.allFinite() || range < 0.20 || range > 8.0) {
          continue;
        }
        // 将点表达为以 base 原点为中心的世界方向向量，使 GICP 平移直接
        // 对应 base 平移，避免原地转向时相机安装臂长被误积累成位移。
        points.push_back(
            world_depth_rotation * point +
            world_base_rotation * base_to_depth_.translation());
      }
    } catch (const std::runtime_error &error) {
      ROS_ERROR_THROTTLE(2.0, "深度点云字段无效：%s", error.what());
      return;
    }
    if (use_adaptive_shared_map_) {
      std::vector<Eigen::Vector3d> structural_vectors;
      structural_vectors.reserve(points.size());
      for (const auto &point : points) {
        const double floor_relative_height = base_height + point.z();
        if (floor_relative_height > 0.22 && floor_relative_height < 1.90) {
          structural_vectors.push_back(point);
        }
      }
      std::lock_guard<std::mutex> shared_lock(shared_map_mutex_);
      adaptive_depth_proposal_ = {};
      adaptive_depth_proposal_stamp_ = message->header.stamp;
      if (!imu_odometry_->initialized()) {
        return;
      }
      if (last_adaptive_output_stamp_.isZero() ||
          message->header.stamp < last_adaptive_output_stamp_) {
        ROS_WARN_THROTTLE(
            2.0,
            "深度候选时间戳早于最近正式定位，拒绝用新位姿配旧点云："
            "depth=%.3f output=%.3f",
            message->header.stamp.toSec(),
            last_adaptive_output_stamp_.toSec());
        return;
      }
      const double frame_interval = last_adaptive_output_stamp_.isZero()
                                        ? std::numeric_limits<double>::quiet_NaN()
                                        : message->header.stamp.toSec() -
                                              last_adaptive_output_stamp_.toSec();
      Eigen::Vector3d translation_prior = adaptive_base_translation_;
      translation_prior.head<2>() += CommandMotionPrior(
          last_adaptive_output_stamp_, message->header.stamp,
          world_base_rotation);
      const double translation_limit =
          frame_interval <= 0.0
              ? std::min(livox_imu_translation_margin_,
                         livox_imu_absolute_translation_cap_)
              : danger_search_localization::PlanarTranslationLimit(
                    frame_interval, livox_imu_maximum_speed_,
                    livox_imu_translation_margin_,
                    livox_imu_absolute_translation_cap_);
      const auto result = imu_odometry_->EstimateFrame(
          structural_vectors, translation_prior, adaptive_base_translation_,
          translation_limit);
      adaptive_depth_proposal_.accepted = result.accepted;
      adaptive_depth_proposal_.translation = result.translation;
      adaptive_depth_proposal_.age_seconds = 0.0;
      adaptive_depth_proposal_.map_generation = result.map_generation;
      Eigen::Vector2d delta = Eigen::Vector2d::Zero();
      if (result.accepted) {
        delta = (result.translation - adaptive_base_translation_).head<2>();
      }
      RecordMapRegistrationDiagnostic(
          "adaptive_rgbd_proposal", message->header.stamp, result, delta,
          &map_registration_diagnostics_);
      ROS_INFO_THROTTLE(
          2.0,
          "共享地图 RGB-D 候选：accepted=%d stamp=%.3f generation=%lu "
          "position=(%.3f, %.3f) corr=%zu",
          result.accepted ? 1 : 0, message->header.stamp.toSec(),
          static_cast<unsigned long>(result.map_generation),
          result.translation.x(), result.translation.y(),
          result.correspondence_count);
      return;
    }
    const Eigen::Vector2d command_prior = CommandMotionPrior(
        last_depth_registration_stamp_, message->header.stamp,
        world_base_rotation);
    const double frame_interval = last_depth_registration_stamp_.isZero()
                                      ? std::numeric_limits<double>::quiet_NaN()
                                      : message->header.stamp.toSec() -
                                            last_depth_registration_stamp_.toSec();
    const auto result = depth_gicp_.RegisterFrame(points, command_prior);
    RecordPlanarRegistrationDiagnostic(
        "rgbd_frame_gicp", message->header.stamp, frame_interval, result,
        &depth_registration_diagnostics_);
    if (result.point_count >= 80) {
      if (diagnostic_frame_details_) {
        std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
        depth_command_total_ += command_prior;
      }
      last_depth_registration_stamp_ = message->header.stamp;
    }
    if (!result.accepted) {
      ROS_WARN_THROTTLE(
          2.0, "RGB-D 平移配准不可用：points=%zu fitness=%.4f",
          result.point_count, result.fitness);
      return;
    }
    if (result.has_delta) {
      LogLargeTranslationIncrement(
          "rgbd_frame_gicp", message->header.stamp, result.delta,
          result.fitness, result.point_count);
    }
    {
      std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
      if (result.has_delta) {
        depth_total_ += result.delta;
      }
      const bool recovering =
          last_depth_accept_stamp_.isZero() ||
          message->header.stamp.toSec() - last_depth_accept_stamp_.toSec() >
              depth_freshness_seconds_;
      if (recovering) {
        depth_fusion_offset_ = fused_planar_position_ - depth_total_;
      }
      fused_planar_position_ = depth_total_ + depth_fusion_offset_;
      last_depth_accept_stamp_ = message->header.stamp;
      fused_at_last_depth_ = fused_planar_position_;
      livox_at_last_depth_ = livox_total_;
      UpdateLivoxScale(message->header.stamp);
    }
  }

  void OnCloud(const sensor_msgs::PointCloud::ConstPtr &message) {
    // Livox 是世界系初始化和最终输出的主时钟。首个有效帧先估计楼面；之后按
    // 后端执行纯 KISS-ICP、IMU 加速度积分旧后端、Livox-IMU 局部地图，或
    // RGB-D 连续里程 + Livox 校正。
    if (message->header.stamp.isZero()) {
      ROS_ERROR_THROTTLE(2.0, "/scan 缺少有效时间戳，拒绝定位帧");
      return;
    }
    if (message->header.frame_id != raw_cloud_frame_) {
      ROS_ERROR_THROTTLE(
          2.0, "/scan frame_id 不符合公开契约：实际=%s，期望=%s",
          message->header.frame_id.c_str(), raw_cloud_frame_.c_str());
      return;
    }
    std::vector<Eigen::Vector3d> points;
    points.reserve(message->points.size());
    for (const auto &point : message->points) {
      const Eigen::Vector3d candidate(point.x, point.y, point.z);
      if (danger_search_localization::IsPointInLocalizationRange(
              candidate, min_range_, max_range_)) {
        points.emplace_back(candidate);
      }
    }
    if (points.size() < 50) {
      ROS_WARN_THROTTLE(2.0, "/scan 有效点不足，跳过定位帧");
      return;
    }

    std::mutex &process_mutex =
        use_adaptive_shared_map_
            ? shared_map_mutex_
            : (use_rgbd_livox_ ? rgbd_scan_mutex_ : processing_mutex_);
    std::unique_lock<std::mutex> process_lock(process_mutex);

    std::optional<ImuSample> current_imu_sample;
    if (use_imu_constrained_ || use_livox_imu_local_map_ ||
        use_adaptive_shared_map_ || use_rgbd_livox_ || stance_health_enabled_) {
      double imu_delta = std::numeric_limits<double>::infinity();
      current_imu_sample = ImuSampleAt(message->header.stamp, &imu_delta);
      if (!current_imu_sample.has_value()) {
        ROS_WARN_THROTTLE(
            2.0,
            "/trunk_imu 与 /scan 时间不匹配，跳过定位帧（最近差 %.3f s）",
            imu_delta);
        return;
      }
    }

    std::optional<danger_search_localization::GroundPlaneEstimate>
        current_ground;
    if (!world_frame_initialized_ || stance_health_enabled_) {
      std::vector<Eigen::Vector3d> base_points;
      base_points.reserve(points.size());
      for (const auto &point : points) {
        base_points.push_back(base_to_livox_ * point);
      }
      current_ground =
          danger_search_localization::EstimateGroundPlane(base_points);
    }

    {
      std::lock_guard<std::mutex> initialization_lock(initialization_mutex_);
      if (!world_frame_initialized_) {
        if (!current_ground.has_value()) {
          ROS_WARN_THROTTLE(
              2.0,
              "楼面证据不足，等待下一份 /scan；不会使用出生高度发布错误世界坐标");
          return;
        }
        const auto &ground = current_ground;
        const double base_up_z = current_imu_sample.has_value()
                                     ? (current_imu_sample->rotation *
                                        Eigen::Vector3d::UnitZ())
                                           .z()
                                     : 1.0;
        const double angular_speed =
            current_imu_sample.has_value()
                ? current_imu_sample->angular_velocity.norm()
                : 0.0;
        const bool stance_ready = stance_initialization_gate_.Update(
            danger_search_localization::StanceObservation{
                ground->distance_from_base, ground->normal_base.z(), base_up_z,
                angular_speed});
        if (!stance_ready) {
          ROS_WARN_THROTTLE(
              2.0,
              "等待站立稳定后初始化定位：base_z=%.3f ground_nz=%.3f "
              "base_up_z=%.3f angular=%.3f stable_frames=%zu",
              ground->distance_from_base, ground->normal_base.z(), base_up_z,
              angular_speed, stance_initialization_gate_.consecutive_frames());
          return;
        }
        const Eigen::Isometry3d world_base_matrix =
            danger_search_localization::BuildWorldBaseFromGroundPlane(
                ground->normal_base, ground->distance_from_base,
                start_pose_.x, start_pose_.y, start_pose_.yaw);
        const SE3d world_base0(
            SO3d(world_base_matrix.linear()), world_base_matrix.translation());
        world_livox0_ = world_base0 * base_to_livox_;
        initial_world_base_rotation_ = world_base0.rotationMatrix();
        initial_world_base_translation_ = world_base0.translation();
        if (use_imu_constrained_ || use_livox_imu_local_map_ ||
            use_adaptive_shared_map_ || use_rgbd_livox_) {
          initial_imu_rotation_ = current_imu_sample->rotation;
          imu_anchor_initialized_ = true;
        }
        if (use_imu_constrained_) {
          imu_motion_predictor_.Reset();
          imu_motion_predictor_.AddSample(
              current_imu_sample->stamp.toSec(), initial_world_base_rotation_,
              current_imu_sample->linear_acceleration,
              current_imu_sample->angular_velocity);
        }
        if (use_rgbd_livox_) {
          fused_planar_position_ = world_base0.translation().head<2>();
          fused_at_last_depth_ = fused_planar_position_;
          depth_fusion_offset_ = fused_planar_position_;
          fusion_initialized_ = true;
        }
        if (use_livox_imu_local_map_) {
          local_map_base_translation_ = world_base0.translation();
        }
        if (use_adaptive_shared_map_) {
          adaptive_base_translation_ = world_base0.translation();
          adaptive_pose_initialized_ = true;
        }
        world_frame_initialized_ = true;
        ROS_INFO(
            "定位世界系已由 Livox 楼面初始化：base_z=%.3f normal=(%.3f, %.3f, %.3f) "
            "inliers=%zu/%zu public_spawn_z=%.3f",
            ground->distance_from_base, ground->normal_base.x(), ground->normal_base.y(),
            ground->normal_base.z(), ground->inlier_count, ground->candidate_count,
            start_pose_.z);
      }
    }

    SE3d world_livox;
    if (use_adaptive_shared_map_) {
      const Eigen::Matrix3d world_base_rotation =
          danger_search_localization::AnchorImuOrientation(
              initial_world_base_rotation_, initial_imu_rotation_,
              current_imu_sample->rotation);
      const Eigen::Matrix3d world_livox_rotation =
          world_base_rotation * base_to_livox_.rotationMatrix();
      const Eigen::Vector3d base_to_livox_world =
          world_base_rotation * base_to_livox_.translation();
      std::vector<Eigen::Vector3d> structural_vectors;
      structural_vectors.reserve(points.size());
      for (const auto &point : points) {
        const Eigen::Vector3d base_centered =
            world_livox_rotation * point + base_to_livox_world;
        const double floor_relative_height =
            adaptive_base_translation_.z() + base_centered.z();
        if (floor_relative_height > 0.22 && floor_relative_height < 1.90 &&
            point.norm() < 12.0) {
          structural_vectors.push_back(base_centered);
        }
      }

      if (!imu_odometry_->initialized()) {
        if (!imu_odometry_->CommitFrame(
                structural_vectors, adaptive_base_translation_,
                imu_odometry_->map_generation())) {
          ROS_WARN_THROTTLE(
              2.0, "共享地图首帧结构不足，等待下一份有效 Livox 帧");
          UpdateAndPublishStanceHealth(
              message->header.stamp, current_imu_sample, current_ground);
          return;
        }
        adaptive_depth_proposal_ = {};
        adaptive_depth_proposal_stamp_ = ros::Time();
        last_adaptive_output_stamp_ = message->header.stamp;
        world_livox = SE3d(
            SO3d(world_livox_rotation),
            adaptive_base_translation_ + base_to_livox_world);
        ROS_INFO(
            "共享地图自适应定位已初始化：generation=%lu map_points=%zu",
            static_cast<unsigned long>(imu_odometry_->map_generation()),
            imu_odometry_->map_point_count());
      } else {
        const double frame_interval =
            message->header.stamp.toSec() - last_adaptive_output_stamp_.toSec();
        if (!std::isfinite(frame_interval) || frame_interval <= 0.0) {
          ROS_WARN_THROTTLE(
              2.0, "共享地图收到非单调 Livox 时间戳，停止发布本帧定位");
          UpdateAndPublishStanceHealth(
              message->header.stamp, current_imu_sample, current_ground);
          return;
        }
        const double translation_limit =
            danger_search_localization::PlanarTranslationLimit(
                frame_interval, livox_imu_maximum_speed_,
                livox_imu_translation_margin_,
                livox_imu_absolute_translation_cap_);
        Eigen::Vector3d livox_prior = adaptive_base_translation_;
        livox_prior.head<2>() += CommandMotionPrior(
            last_adaptive_output_stamp_, message->header.stamp,
            world_base_rotation);
        const auto livox_result = imu_odometry_->EstimateFrame(
            structural_vectors, livox_prior, adaptive_base_translation_,
            translation_limit);
        danger_search_localization::TranslationProposal livox_proposal;
        livox_proposal.accepted = livox_result.accepted;
        livox_proposal.translation = livox_result.translation;
        livox_proposal.age_seconds = 0.0;
        livox_proposal.map_generation = livox_result.map_generation;

        auto rgbd_proposal = adaptive_depth_proposal_;
        const ros::Time rgbd_stamp = adaptive_depth_proposal_stamp_;
        adaptive_depth_proposal_ = {};
        adaptive_depth_proposal_stamp_ = ros::Time();
        if (!rgbd_stamp.isZero()) {
          rgbd_proposal.age_seconds =
              message->header.stamp.toSec() - rgbd_stamp.toSec();
          if (rgbd_proposal.accepted &&
              std::isfinite(rgbd_proposal.age_seconds) &&
              rgbd_proposal.age_seconds >= 0.0) {
            rgbd_proposal.translation.head<2>() += CommandMotionPrior(
                rgbd_stamp, message->header.stamp, world_base_rotation);
            if (std::isfinite(translation_limit) &&
                (rgbd_proposal.translation - adaptive_base_translation_)
                        .head<2>()
                        .norm() > translation_limit) {
              rgbd_proposal.accepted = false;
            }
          }
        }

        const auto decision =
            danger_search_localization::DecideAdaptiveTranslation(
                rgbd_proposal, livox_proposal,
                imu_odometry_->map_generation(), adaptive_selector_config_);
        Eigen::Vector2d livox_delta = Eigen::Vector2d::Zero();
        if (livox_result.accepted) {
          livox_delta =
              (livox_result.translation - adaptive_base_translation_).head<2>();
        }
        RecordMapRegistrationDiagnostic(
            "adaptive_livox_proposal", message->header.stamp, livox_result,
            livox_delta, &map_registration_diagnostics_);

        if (!decision.accepted) {
          ++adaptive_both_invalid_count_;
          ROS_WARN_THROTTLE(
              1.0,
              "共享地图两路平移候选均无效，停止更新 odom："
              "livox_status=%s rgbd_age=%.3f generation=%lu count=%lu",
              danger_search_localization::ImuConstrainedOdometryStatusName(
                  livox_result.status),
              rgbd_proposal.age_seconds,
              static_cast<unsigned long>(imu_odometry_->map_generation()),
              static_cast<unsigned long>(adaptive_both_invalid_count_));
          UpdateAndPublishStanceHealth(
              message->header.stamp, current_imu_sample, current_ground);
          return;
        }

        bool map_committed = false;
        if (decision.commit_livox_map) {
          map_committed = imu_odometry_->CommitFrame(
              structural_vectors, decision.translation,
              livox_result.map_generation);
          if (!map_committed) {
            ROS_ERROR_THROTTLE(
                1.0,
                "共享地图事务提交失败，停止发布本帧定位以避免位姿和地图分叉");
            UpdateAndPublishStanceHealth(
                message->header.stamp, current_imu_sample, current_ground);
            return;
          }
        }

        const Eigen::Vector2d selected_delta =
            (decision.translation - adaptive_base_translation_).head<2>();
        adaptive_base_translation_ = decision.translation;
        last_adaptive_output_stamp_ = message->header.stamp;
        if (decision.source ==
            danger_search_localization::AdaptiveTranslationSource::kRgbd) {
          ++adaptive_rgbd_selected_count_;
        } else {
          ++adaptive_livox_selected_count_;
        }
        LogLargeTranslationIncrement(
            "adaptive_shared_map", message->header.stamp, selected_delta,
            std::numeric_limits<double>::quiet_NaN(),
            livox_result.correspondence_count);
        ROS_INFO_THROTTLE(
            1.0,
            "共享地图自适应定位：source=%s reason=%s position=(%.3f, %.3f) "
            "delta=(%.3f, %.3f) divergence=%.3f rgbd_age=%.3f "
            "generation=%lu map_commit=%d selected=(rgbd:%lu livox:%lu) "
            "both_invalid=%lu",
            danger_search_localization::AdaptiveTranslationSourceName(
                decision.source),
            danger_search_localization::AdaptiveTranslationReasonName(
                decision.reason),
            adaptive_base_translation_.x(), adaptive_base_translation_.y(),
            selected_delta.x(), selected_delta.y(), decision.source_divergence,
            rgbd_proposal.age_seconds,
            static_cast<unsigned long>(imu_odometry_->map_generation()),
            map_committed ? 1 : 0,
            static_cast<unsigned long>(adaptive_rgbd_selected_count_),
            static_cast<unsigned long>(adaptive_livox_selected_count_),
            static_cast<unsigned long>(adaptive_both_invalid_count_));
        world_livox = SE3d(
            SO3d(world_livox_rotation),
            adaptive_base_translation_ + base_to_livox_world);
      }
    } else if (use_livox_imu_local_map_) {
      const Eigen::Matrix3d world_base_rotation =
          danger_search_localization::AnchorImuOrientation(
              initial_world_base_rotation_, initial_imu_rotation_,
              current_imu_sample->rotation);
      const Eigen::Matrix3d world_livox_rotation =
          world_base_rotation * base_to_livox_.rotationMatrix();
      const Eigen::Vector3d base_to_livox_world =
          world_base_rotation * base_to_livox_.translation();
      std::vector<Eigen::Vector3d> structural_vectors;
      structural_vectors.reserve(points.size());
      for (const auto &point : points) {
        // 以 base 为配准中心，IMU 只负责旋转；局部地图优化变量始终是 base
        // 平移，避免原地转向时 Livox 安装臂长形成伪平移。
        const Eigen::Vector3d base_centered =
            world_livox_rotation * point + base_to_livox_world;
        const double floor_relative_height =
            local_map_base_translation_.z() + base_centered.z();
        if (floor_relative_height > 0.22 && floor_relative_height < 1.90 &&
            point.norm() < 12.0) {
          structural_vectors.push_back(base_centered);
        }
      }
      const double frame_interval = last_local_map_accept_stamp_.isZero()
                                        ? std::numeric_limits<double>::quiet_NaN()
                                        : message->header.stamp.toSec() -
                                              last_local_map_accept_stamp_.toSec();
      Eigen::Vector3d translation_prior = local_map_base_translation_;
      translation_prior.head<2>() += CommandMotionPrior(
          last_local_map_accept_stamp_, message->header.stamp,
          world_base_rotation);
      const double translation_limit =
          danger_search_localization::PlanarTranslationLimit(
              frame_interval, livox_imu_maximum_speed_,
              livox_imu_translation_margin_,
              livox_imu_absolute_translation_cap_);
      const auto result = imu_odometry_->RegisterFrame(
          structural_vectors, translation_prior, translation_limit);
      Eigen::Vector2d accepted_delta = Eigen::Vector2d::Zero();
      if (result.accepted) {
        accepted_delta = result.translation.head<2>() -
                         local_map_base_translation_.head<2>();
      }
      RecordMapRegistrationDiagnostic(
          "livox_imu_local_map", message->header.stamp, result,
          accepted_delta, &map_registration_diagnostics_);
      if (result.accepted) {
        local_map_base_translation_ = result.translation;
        last_local_map_accept_stamp_ = message->header.stamp;
        LogLargeTranslationIncrement(
            "livox_imu_local_map", message->header.stamp, accepted_delta,
            std::numeric_limits<double>::quiet_NaN(),
            result.correspondence_count);
      } else {
        ROS_WARN_THROTTLE(
            2.0,
            "Livox-IMU 局部地图拒绝当前平移，保持上一已接受位姿："
            "status=%s corr=%zu limit=%.3f",
            danger_search_localization::ImuConstrainedOdometryStatusName(
                result.status),
            result.correspondence_count, translation_limit);
      }
      world_livox = SE3d(
          SO3d(world_livox_rotation),
          local_map_base_translation_ + base_to_livox_world);
      ROS_INFO_THROTTLE(
          2.0,
          "Livox-IMU 局部地图定位：accepted=%d position=(%.3f, %.3f) "
          "delta=(%.3f, %.3f) corr=%zu",
          result.accepted ? 1 : 0, local_map_base_translation_.x(),
          local_map_base_translation_.y(), accepted_delta.x(),
          accepted_delta.y(), result.correspondence_count);
    } else if (use_rgbd_livox_) {
      const Eigen::Matrix3d world_base_rotation =
          danger_search_localization::AnchorImuOrientation(
              initial_world_base_rotation_, initial_imu_rotation_,
              current_imu_sample->rotation);
      const Eigen::Matrix3d world_livox_rotation =
          world_base_rotation * base_to_livox_.rotationMatrix();
      std::vector<Eigen::Vector3d> oriented_points;
      oriented_points.reserve(points.size());
      for (const auto &point : points) {
        // 与深度点云一致，以 base 原点为配准中心，消除传感器安装臂长在
        // 转向时产生的伪平移。
        oriented_points.push_back(
            world_livox_rotation * point +
            world_base_rotation * base_to_livox_.translation());
      }
      const Eigen::Vector2d command_prior = CommandMotionPrior(
          last_livox_registration_stamp_, message->header.stamp,
          world_base_rotation);
      const double frame_interval = last_livox_registration_stamp_.isZero()
                                        ? std::numeric_limits<double>::quiet_NaN()
                                        : message->header.stamp.toSec() -
                                              last_livox_registration_stamp_.toSec();
      const auto result = livox_gicp_.RegisterFrame(
          oriented_points, command_prior);
      RecordPlanarRegistrationDiagnostic(
          "livox_frame_gicp", message->header.stamp, frame_interval, result,
          &livox_registration_diagnostics_);
      if (result.accepted && result.has_delta) {
        LogLargeTranslationIncrement(
            "livox_frame_gicp", message->header.stamp, result.delta,
            result.fitness, result.point_count);
      }
      if (result.point_count >= 80) {
        last_livox_registration_stamp_ = message->header.stamp;
      }
      bool depth_fresh = false;
      Eigen::Vector2d fused_position;
      Eigen::Vector2d depth_total;
      Eigen::Vector2d livox_total;
      double livox_scale = 1.0;
      ros::Time depth_accept_stamp;
      {
        std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
        if (result.accepted && result.has_delta) {
          livox_total_ += result.delta;
        }
        depth_fresh =
            !last_depth_accept_stamp_.isZero() &&
            std::abs(message->header.stamp.toSec() - last_depth_accept_stamp_.toSec()) <=
                depth_freshness_seconds_;
        if (!depth_fresh) {
          fused_planar_position_ =
              fused_at_last_depth_ +
              livox_scale_ * (livox_total_ - livox_at_last_depth_);
        }
        fused_position = fused_planar_position_;
        depth_total = depth_total_;
        livox_total = livox_total_;
        livox_scale = livox_scale_;
        depth_accept_stamp = last_depth_accept_stamp_;
      }
      RecordDepthFreshnessDiagnostic(
          message->header.stamp, depth_fresh, depth_accept_stamp);
      // RGB-D 帧间配准提供连续先验，但快速转向后会累积平移误差。
      // 使用 360° Livox 的竖向结构对局部扫描地图校正该先验；结构不足时
      // 保留原先验，不再像纯 IMU 积分后端那样让定位发散或冻结。
      Eigen::Vector3d base_translation_prior =
          initial_world_base_translation_;
      base_translation_prior.head<2>() = fused_position;
      const Eigen::Vector3d sensor_translation_prior =
          base_translation_prior +
          world_base_rotation * base_to_livox_.translation();
      std::vector<Eigen::Vector3d> structural_vectors;
      structural_vectors.reserve(points.size());
      for (const auto &point : points) {
        const Eigen::Vector3d oriented = world_livox_rotation * point;
        const double floor_relative_height =
            sensor_translation_prior.z() + oriented.z();
        if (floor_relative_height > 0.22 && floor_relative_height < 1.90 &&
            point.norm() < 12.0) {
          structural_vectors.push_back(oriented);
        }
      }
      const auto map_result = imu_odometry_->RegisterFrame(
          structural_vectors, sensor_translation_prior);
      Eigen::Vector2d correction = Eigen::Vector2d::Zero();
      if (map_result.accepted) {
        const Eigen::Vector3d corrected_base_translation =
            map_result.translation -
            world_base_rotation * base_to_livox_.translation();
        correction =
            corrected_base_translation.head<2>() - fused_position;
      }
      RecordMapRegistrationDiagnostic(
          "livox_scan_to_map", message->header.stamp, map_result, correction,
          &map_registration_diagnostics_);
      if (map_result.accepted) {
        LogLargeTranslationIncrement(
            "livox_scan_to_map", message->header.stamp, correction,
            std::numeric_limits<double>::quiet_NaN(),
            map_result.correspondence_count);
        {
          std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
          // 深度回调可在扫描匹配期间推进；把校正作为同一世界系平移量
          // 叠加到最新状态，避免用较旧快照覆盖新到达的深度增量。
          fused_planar_position_ += correction;
          depth_fusion_offset_ += correction;
          fused_at_last_depth_ += correction;
          if (diagnostic_frame_details_) {
            map_correction_total_ += correction;
          }
          livox_at_last_depth_ = livox_total_;
          fused_position = fused_planar_position_;
          depth_total = depth_total_;
          livox_total = livox_total_;
        }
        ROS_INFO_THROTTLE(
            2.0,
            "RGB-D/Livox 局部地图校正：correction=(%.3f, %.3f) corr=%zu",
            correction.x(), correction.y(), map_result.correspondence_count);
      }
      RecordMotionConsistencyDiagnostic(
          message->header.stamp, depth_fresh,
          YawFromRotation(world_base_rotation));
      Eigen::Vector3d world_base_translation =
          initial_world_base_translation_;
      world_base_translation.head<2>() = fused_position;
      const Eigen::Vector3d world_livox_translation =
          world_base_translation +
          world_base_rotation * base_to_livox_.translation();
      world_livox =
          SE3d(SO3d(world_livox_rotation), world_livox_translation);
      ROS_INFO_THROTTLE(
          2.0,
          "RGB-D/Livox 定位：depth_fresh=%d position=(%.3f, %.3f) "
          "livox_scale=%.3f depth_total=(%.3f, %.3f) livox_total=(%.3f, %.3f)",
          depth_fresh ? 1 : 0, fused_position.x(), fused_position.y(),
          livox_scale, depth_total.x(), depth_total.y(), livox_total.x(),
          livox_total.y());
    } else if (use_imu_constrained_) {
      if (!imu_anchor_initialized_) {
        initial_imu_rotation_ = current_imu_sample->rotation;
        initial_world_base_rotation_ = world_livox0_.rotationMatrix() *
                                       base_to_livox_.rotationMatrix().transpose();
        imu_anchor_initialized_ = true;
        imu_motion_predictor_.Reset();
      }
      const Eigen::Matrix3d world_base_rotation =
          danger_search_localization::AnchorImuOrientation(
              initial_world_base_rotation_, initial_imu_rotation_,
              current_imu_sample->rotation);
      const Eigen::Matrix3d world_livox_rotation =
          world_base_rotation * base_to_livox_.rotationMatrix();
      Eigen::Vector3d inertial_base_translation =
          initial_world_base_translation_;
      if (imu_motion_predictor_.calibrated()) {
        inertial_base_translation.head<2>() +=
            imu_motion_predictor_.displacement();
      }
      const Eigen::Vector3d inertial_sensor_translation =
          inertial_base_translation +
          world_base_rotation * base_to_livox_.translation();
      std::vector<Eigen::Vector3d> structural_vectors;
      structural_vectors.reserve(points.size());
      const double sensor_height = inertial_sensor_translation.z();
      for (const auto &point : points) {
        const Eigen::Vector3d oriented = world_livox_rotation * point;
        const double world_height = sensor_height + oriented.z();
        // 楼面会使平面平移退化，天花板也缺少 xy 约束；只使用室内竖向结构。
        if (world_height > 0.22 && world_height < 1.90 &&
            point.norm() < 12.0) {
          structural_vectors.push_back(oriented);
        }
      }
      Eigen::Vector3d translation_prior =
          inertial_sensor_translation + scan_translation_correction_;
      const auto result = imu_odometry_->RegisterFrame(
          structural_vectors, translation_prior);
      if (!result.accepted) {
        ROS_WARN_THROTTLE(
            2.0,
            "IMU 约束定位的竖向结构对应不足，跳过定位帧（对应数=%zu）",
            result.correspondence_count);
        return;
      }
      scan_translation_correction_.head<2>() =
          result.translation.head<2>() -
          inertial_sensor_translation.head<2>();
      ROS_INFO_THROTTLE(
          2.0,
          "IMU 约束定位：imu_ready=%d displacement=(%.3f, %.3f) "
          "velocity=(%.3f, %.3f) scan_correction=(%.3f, %.3f) corr=%zu",
          imu_motion_predictor_.calibrated() ? 1 : 0,
          imu_motion_predictor_.displacement().x(),
          imu_motion_predictor_.displacement().y(),
          imu_motion_predictor_.velocity().x(),
          imu_motion_predictor_.velocity().y(),
          scan_translation_correction_.x(),
          scan_translation_correction_.y(), result.correspondence_count);
      world_livox = SE3d(SO3d(world_livox_rotation), result.translation);
    } else {
      odometry_.RegisterFrame(points);
      const SE3d local_livox = odometry_.poses().back();
      world_livox = world_livox0_ * local_livox;
    }
    const SE3d world_base = world_livox * base_to_livox_.inverse();
    if (!UpdateAndPublishStanceHealth(
            message->header.stamp, current_imu_sample, current_ground)) {
      return;
    }
    PublishOdometry(message->header.stamp, world_base);
    PublishRegisteredCloud(message->header.stamp, world_livox, points);
  }

  void PublishOdometry(const ros::Time &stamp, const SE3d &world_base) {
    // 位姿使用 world->base；速度由固定窗口的 world 位姿差分后转到 base，避免
    // 单帧配准抖动被 10 Hz 差分放大。所有字段沿用输入点云时间戳。
    nav_msgs::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.child_frame_id = "base";
    const Eigen::Vector3d translation = world_base.translation();
    const Eigen::Quaterniond orientation(world_base.rotationMatrix());
    message.pose.pose.position.x = translation.x();
    message.pose.pose.position.y = translation.y();
    message.pose.pose.position.z = translation.z();
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();

    const auto velocity = velocity_estimator_.Update(
        stamp.toSec(), translation, YawFromRotation(world_base.rotationMatrix()));
    if (velocity.valid) {
      // 里程计 twist 在 child_frame_id="base" 下表达，需将世界系速度转换到 base 系。
      const Eigen::Vector3d velocity_base =
          danger_search_localization::WorldVelocityToBase(
              velocity.linear_world, world_base.rotationMatrix());
      message.twist.twist.linear.x = velocity_base.x();
      message.twist.twist.linear.y = velocity_base.y();
      message.twist.twist.linear.z = velocity_base.z();
      message.twist.twist.angular.z = velocity.yaw_rate;
    }
    odom_publisher_.publish(message);
  }

  void PublishRegisteredCloud(const ros::Time &stamp, const SE3d &world_livox,
                              const std::vector<Eigen::Vector3d> &points) {
    // 将本帧已过滤 Livox 点逐点变换到 world，供 Mapper 使用；该内部话题不增加
    // 新的外部信息源，坐标完全来自同一定位闭包。
    sensor_msgs::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    for (const auto &point : points) {
      const Eigen::Vector3d world_point = world_livox * point;
      *x = static_cast<float>(world_point.x());
      *y = static_cast<float>(world_point.y());
      *z = static_cast<float>(world_point.z());
      ++x;
      ++y;
      ++z;
    }
    cloud_publisher_.publish(message);
  }

  void RecordPlanarRegistrationDiagnostic(
      const char *source, const ros::Time &stamp,
      double frame_interval,
      const danger_search_localization::PlanarGicpResult &result,
      PlanarRegistrationDiagnosticStats *stats) const {
    if (diagnostic_frame_details_) {
      ROS_INFO(
          "localization_planar_registration_frame source=%s scan_stamp=%.6f "
          "frame_interval=%.6f status=%s accepted=%d has_delta=%d "
          "candidate=(%.6f, %.6f) applied=(%.6f, %.6f) "
          "candidate_yaw=%.8f candidate_rotation_angle=%.8f "
          "fitness=%.8f points=%zu",
          source, stamp.toSec(), frame_interval,
          danger_search_localization::PlanarGicpStatusName(result.status),
          result.accepted ? 1 : 0, result.has_delta ? 1 : 0,
          result.candidate_delta.x(), result.candidate_delta.y(),
          result.delta.x(), result.delta.y(), result.candidate_yaw,
          result.candidate_rotation_angle, result.fitness, result.point_count);
    }
    const std::size_t status_index = static_cast<std::size_t>(result.status);
    if (status_index < stats->status_counts.size()) {
      ++stats->status_counts[status_index];
    }
    const double candidate_norm = result.candidate_delta.norm();
    if (std::isfinite(candidate_norm)) {
      stats->maximum_candidate_norm =
          std::max(stats->maximum_candidate_norm, candidate_norm);
    }
    if (std::isfinite(result.candidate_rotation_angle)) {
      stats->maximum_candidate_rotation_angle = std::max(
          stats->maximum_candidate_rotation_angle,
          result.candidate_rotation_angle);
    }
    if (stats->interval_start.isZero()) {
      stats->interval_start = stamp;
      return;
    }
    if (stamp.toSec() - stats->interval_start.toSec() <
        diagnostic_summary_period_) {
      return;
    }
    const auto count = [stats](
                           danger_search_localization::PlanarGicpStatus status) {
      return static_cast<unsigned long long>(
          stats->status_counts[static_cast<std::size_t>(status)]);
    };
    ROS_INFO(
        "localization_registration_summary source=%s stamp=%.6f "
        "accepted=%llu initial=%llu invalid_prior=%llu "
        "insufficient_points=%llu not_converged=%llu "
        "nonfinite_delta=%llu nonfinite_fitness=%llu "
        "translation_limit=%llu fitness_limit=%llu "
        "maximum_candidate_norm=%.4f maximum_candidate_rotation_angle=%.6f "
        "latest_status=%s "
        "latest_candidate=(%.4f, %.4f) latest_applied=(%.4f, %.4f) "
        "latest_candidate_yaw=%.6f latest_candidate_rotation_angle=%.6f "
        "latest_fitness=%.6f latest_points=%zu",
        source, stamp.toSec(),
        count(danger_search_localization::PlanarGicpStatus::kAccepted),
        count(danger_search_localization::PlanarGicpStatus::kInitialFrame),
        count(danger_search_localization::PlanarGicpStatus::kInvalidPrior),
        count(danger_search_localization::PlanarGicpStatus::kInsufficientPoints),
        count(danger_search_localization::PlanarGicpStatus::kNotConverged),
        count(danger_search_localization::PlanarGicpStatus::kNonFiniteDelta),
        count(danger_search_localization::PlanarGicpStatus::kNonFiniteFitness),
        count(danger_search_localization::PlanarGicpStatus::kTranslationLimit),
        count(danger_search_localization::PlanarGicpStatus::kFitnessLimit),
        stats->maximum_candidate_norm,
        stats->maximum_candidate_rotation_angle,
        danger_search_localization::PlanarGicpStatusName(result.status),
        result.candidate_delta.x(), result.candidate_delta.y(),
        result.delta.x(), result.delta.y(), result.candidate_yaw,
        result.candidate_rotation_angle, result.fitness, result.point_count);
    stats->status_counts.fill(0);
    stats->maximum_candidate_norm = 0.0;
    stats->maximum_candidate_rotation_angle = 0.0;
    stats->interval_start = stamp;
  }

  void RecordMapRegistrationDiagnostic(
      const char *source, const ros::Time &stamp,
      const danger_search_localization::ImuConstrainedOdometryResult &result,
      const Eigen::Vector2d &applied_correction,
      MapRegistrationDiagnosticStats *stats) const {
    if (diagnostic_frame_details_) {
      ROS_INFO(
          "localization_map_registration_frame scan_stamp=%.6f source=%s "
          "status=%s accepted=%d raw=(%.6f, %.6f) "
          "applied=(%.6f, %.6f) clamped=%d source_points=%zu matched=%zu "
          "retained=%zu map_points=%zu iterations=%d "
          "residual_p50=%.6f residual_p70=%.6f residual_p90=%.6f "
          "residual_mean=(%.6f, %.6f) residual_median=(%.6f, %.6f) "
          "residual_mad=(%.6f, %.6f) residual_positive=(%.6f, %.6f) "
          "residual_negative=(%.6f, %.6f) covariance_eigen=(%.9f, %.9f) "
          "anisotropy=%.6f",
          stamp.toSec(), source,
          danger_search_localization::ImuConstrainedOdometryStatusName(
              result.status),
          result.accepted ? 1 : 0, result.unclamped_correction.x(),
          result.unclamped_correction.y(), applied_correction.x(),
          applied_correction.y(), result.correction_was_clamped ? 1 : 0,
          result.source_point_count, result.matched_correspondence_count,
          result.correspondence_count, result.map_point_count,
          result.iteration_count, result.residual_p50, result.residual_p70,
          result.residual_p90, result.residual_mean.x(),
          result.residual_mean.y(), result.residual_median.x(),
          result.residual_median.y(), result.residual_mad.x(),
          result.residual_mad.y(), result.residual_positive_fraction.x(),
          result.residual_positive_fraction.y(),
          result.residual_negative_fraction.x(),
          result.residual_negative_fraction.y(),
          result.residual_covariance_min_eigenvalue,
          result.residual_covariance_max_eigenvalue,
          result.residual_anisotropy);
    }
    const std::size_t status_index = static_cast<std::size_t>(result.status);
    if (status_index < stats->status_counts.size()) {
      ++stats->status_counts[status_index];
    }
    const double raw_norm = result.unclamped_correction.norm();
    const double applied_norm = applied_correction.norm();
    if (std::isfinite(raw_norm)) {
      stats->maximum_unclamped_norm =
          std::max(stats->maximum_unclamped_norm, raw_norm);
    }
    if (std::isfinite(applied_norm)) {
      stats->maximum_applied_norm =
          std::max(stats->maximum_applied_norm, applied_norm);
    }
    if (result.correction_was_clamped) {
      ++stats->clamped_count;
    }
    if (stats->interval_start.isZero()) {
      stats->interval_start = stamp;
      return;
    }
    if (stamp.toSec() - stats->interval_start.toSec() <
        diagnostic_summary_period_) {
      return;
    }
    const auto count = [stats](
                           danger_search_localization::ImuConstrainedOdometryStatus status) {
      return static_cast<unsigned long long>(
          stats->status_counts[static_cast<std::size_t>(status)]);
    };
    ROS_INFO(
        "localization_map_registration_summary source=%s stamp=%.6f "
        "accepted=%llu initial=%llu insufficient_source=%llu "
        "insufficient_correspondences=%llu translation_limit=%llu clamped=%llu "
        "maximum_unclamped_norm=%.4f maximum_applied_norm=%.4f "
        "latest_status=%s latest_unclamped=(%.4f, %.4f) "
        "latest_applied=(%.4f, %.4f) latest_correspondences=%zu "
        "latest_source=%zu latest_matched=%zu latest_map=%zu "
        "latest_iterations=%d latest_residual_p70=%.5f "
        "latest_residual_mad=(%.5f, %.5f) latest_anisotropy=%.5f",
        source, stamp.toSec(),
        count(danger_search_localization::ImuConstrainedOdometryStatus::kAccepted),
        count(danger_search_localization::ImuConstrainedOdometryStatus::kInitialFrame),
        count(danger_search_localization::ImuConstrainedOdometryStatus::kInsufficientSource),
        count(danger_search_localization::ImuConstrainedOdometryStatus::kInsufficientCorrespondences),
        count(danger_search_localization::ImuConstrainedOdometryStatus::kTranslationLimit),
        static_cast<unsigned long long>(stats->clamped_count),
        stats->maximum_unclamped_norm, stats->maximum_applied_norm,
        danger_search_localization::ImuConstrainedOdometryStatusName(
            result.status),
        result.unclamped_correction.x(), result.unclamped_correction.y(),
        applied_correction.x(), applied_correction.y(),
        result.correspondence_count, result.source_point_count,
        result.matched_correspondence_count, result.map_point_count,
        result.iteration_count, result.residual_p70, result.residual_mad.x(),
        result.residual_mad.y(), result.residual_anisotropy);
    stats->status_counts.fill(0);
    stats->maximum_unclamped_norm = 0.0;
    stats->maximum_applied_norm = 0.0;
    stats->clamped_count = 0;
    stats->interval_start = stamp;
  }

  void RecordMotionConsistencyDiagnostic(
      const ros::Time &stamp, bool depth_fresh, double yaw) {
    if (!diagnostic_frame_details_) {
      return;
    }
    Eigen::Vector2d depth_total;
    Eigen::Vector2d livox_total;
    Eigen::Vector2d map_correction_total;
    Eigen::Vector2d fused_position;
    Eigen::Vector2d command_total;
    {
      std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
      depth_total = depth_total_;
      livox_total = livox_total_;
      map_correction_total = map_correction_total_;
      fused_position = fused_planar_position_;
      command_total = depth_command_total_;
    }
    auto &state = motion_consistency_diagnostics_;
    if (!state.initialized) {
      state.initialized = true;
      state.interval_start = stamp;
      state.depth_anchor = depth_total;
      state.livox_anchor = livox_total;
      state.map_correction_anchor = map_correction_total;
      state.fused_anchor = fused_position;
      state.command_anchor = command_total;
      state.previous_yaw = yaw;
      return;
    }
    const double yaw_delta = std::atan2(
        std::sin(yaw - state.previous_yaw),
        std::cos(yaw - state.previous_yaw));
    state.absolute_yaw_change += std::abs(yaw_delta);
    state.previous_yaw = yaw;
    ++state.frame_count;
    state.depth_fresh_count += depth_fresh ? 1U : 0U;
    const double duration = stamp.toSec() - state.interval_start.toSec();
    if (duration < diagnostic_summary_period_) {
      return;
    }

    const Eigen::Vector2d depth_delta = depth_total - state.depth_anchor;
    const Eigen::Vector2d livox_delta = livox_total - state.livox_anchor;
    const Eigen::Vector2d map_delta =
        map_correction_total - state.map_correction_anchor;
    const Eigen::Vector2d fused_delta = fused_position - state.fused_anchor;
    const Eigen::Vector2d command_delta = command_total - state.command_anchor;
    const Eigen::Vector2d divergence = livox_delta - depth_delta;
    const auto axis_relation = [](double correction, double difference) {
      if (std::abs(correction) < 1e-9 || std::abs(difference) < 1e-9) {
        return 0;
      }
      return correction * difference > 0.0 ? 1 : -1;
    };
    const auto axis_ratio = [](double correction, double difference) {
      return std::abs(difference) > 1e-9
                 ? std::abs(correction / difference)
                 : std::numeric_limits<double>::quiet_NaN();
    };
    const double fresh_ratio = state.frame_count > 0
                                   ? static_cast<double>(state.depth_fresh_count) /
                                         static_cast<double>(state.frame_count)
                                   : 0.0;
    ROS_INFO(
        "localization_motion_consistency_summary scan_stamp=%.6f "
        "duration=%.6f depth=(%.6f, %.6f) livox=(%.6f, %.6f) "
        "scan_to_map=(%.6f, %.6f) fused=(%.6f, %.6f) "
        "command=(%.6f, %.6f) divergence=(%.6f, %.6f) "
        "axis_relation=(%d, %d) axis_ratio=(%.6f, %.6f) "
        "imu_abs_yaw=%.6f depth_fresh_ratio=%.6f frames=%zu",
        stamp.toSec(), duration, depth_delta.x(), depth_delta.y(),
        livox_delta.x(), livox_delta.y(), map_delta.x(), map_delta.y(),
        fused_delta.x(), fused_delta.y(), command_delta.x(),
        command_delta.y(), divergence.x(), divergence.y(),
        axis_relation(map_delta.x(), divergence.x()),
        axis_relation(map_delta.y(), divergence.y()),
        axis_ratio(map_delta.x(), divergence.x()),
        axis_ratio(map_delta.y(), divergence.y()),
        state.absolute_yaw_change, fresh_ratio, state.frame_count);

    state.interval_start = stamp;
    state.depth_anchor = depth_total;
    state.livox_anchor = livox_total;
    state.map_correction_anchor = map_correction_total;
    state.fused_anchor = fused_position;
    state.command_anchor = command_total;
    state.absolute_yaw_change = 0.0;
    state.frame_count = 0;
    state.depth_fresh_count = 0;
  }

  void RecordDepthFreshnessDiagnostic(
      const ros::Time &stamp, bool depth_fresh,
      const ros::Time &last_depth_accept_stamp) {
    if (!depth_freshness_state_initialized_) {
      depth_freshness_state_initialized_ = true;
      previous_depth_fresh_ = depth_fresh;
      if (!depth_fresh) {
        depth_stale_since_ = stamp;
      }
      return;
    }
    if (depth_fresh == previous_depth_fresh_) {
      return;
    }
    const double last_accept_age = last_depth_accept_stamp.isZero()
                                       ? std::numeric_limits<double>::infinity()
                                       : stamp.toSec() -
                                             last_depth_accept_stamp.toSec();
    if (depth_fresh) {
      const double stale_duration = depth_stale_since_.isZero()
                                        ? 0.0
                                        : std::max(
                                              0.0, stamp.toSec() -
                                                       depth_stale_since_.toSec());
      ROS_INFO(
          "localization_depth_freshness_transition stamp=%.6f fresh=1 "
          "stale_duration=%.4f last_accept_age=%.4f",
          stamp.toSec(), stale_duration, last_accept_age);
      depth_stale_since_ = ros::Time();
    } else {
      depth_stale_since_ = stamp;
      ROS_WARN(
          "localization_depth_freshness_transition stamp=%.6f fresh=0 "
          "last_accept_age=%.4f",
          stamp.toSec(), last_accept_age);
    }
    previous_depth_fresh_ = depth_fresh;
  }

  void LogLargeTranslationIncrement(
      const char *source, const ros::Time &stamp,
      const Eigen::Vector2d &increment, double fitness,
      std::size_t evidence_count) const {
    const double norm = increment.norm();
    if (!std::isfinite(norm) ||
        norm < diagnostic_large_translation_threshold_) {
      return;
    }
    // 只记录稀疏异常事件，不参与融合决策，避免逐帧日志改变控制实时率。
    ROS_WARN(
        "localization_large_translation_increment source=%s stamp=%.6f "
        "delta=(%.4f, %.4f) norm=%.4f fitness=%.6f evidence=%zu",
        source, stamp.toSec(), increment.x(), increment.y(), norm, fitness,
        evidence_count);
  }

  bool UpdateAndPublishStanceHealth(
      const ros::Time &stamp,
      const std::optional<ImuSample> &current_imu_sample,
      const std::optional<danger_search_localization::GroundPlaneEstimate>
          &ground) {
    if (!stance_health_enabled_) {
      std_msgs::Bool ready_msg;
      ready_msg.data = true;
      ready_publisher_.publish(ready_msg);
      return true;
    }
    // 姿态健康只依赖公开 /scan 楼面证据和 /trunk_imu。建立世界系前保持
    // false；初始化后每帧更新。明确越界是异常，短时缺失是有界 UNKNOWN，
    // 避免稀疏楼面回波把直立机器人误判为跌倒。
    if (!world_frame_initialized_) {
      return false;
    }
    const double base_up_z =
        current_imu_sample.has_value()
            ? (current_imu_sample->rotation * Eigen::Vector3d::UnitZ()).z()
            : 1.0;
    const double angular_speed =
        current_imu_sample.has_value()
            ? current_imu_sample->angular_velocity.norm()
            : std::numeric_limits<double>::quiet_NaN();
    const double base_height =
        ground.has_value()
            ? ground->distance_from_base
            : std::numeric_limits<double>::quiet_NaN();
    const double ground_normal_z =
        ground.has_value()
            ? ground->normal_base.z()
            : std::numeric_limits<double>::quiet_NaN();
    const bool healthy = stance_health_monitor_.Update(
        danger_search_localization::StanceObservation{
            base_height, ground_normal_z, base_up_z, angular_speed});
    std_msgs::Bool health_msg;
    health_msg.data = healthy;
    health_publisher_.publish(health_msg);
    if (!healthy) {
      std_msgs::Bool not_ready;
      not_ready.data = false;
      ready_publisher_.publish(not_ready);
      ROS_WARN_THROTTLE(
          2.0,
          "定位姿态健康异常：base_up_z=%.3f angular=%.3f "
          "base_z=%.3f ground_nz=%.3f good_frames=%zu bad_frames=%zu "
          "unknown_frames=%zu；已撤销 ready",
          base_up_z, angular_speed, base_height, ground_normal_z,
          stance_health_monitor_.consecutive_good_frames(),
          stance_health_monitor_.consecutive_bad_frames(),
          stance_health_monitor_.consecutive_unknown_frames());
    } else {
      std_msgs::Bool ready_msg;
      ready_msg.data = true;
      ready_publisher_.publish(ready_msg);
    }
    return healthy;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber depth_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Publisher odom_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Publisher ready_publisher_;
  ros::Publisher health_publisher_;
  kiss_icp::pipeline::KissICP odometry_;
  std::unique_ptr<danger_search_localization::ImuConstrainedOdometry>
      imu_odometry_;
  StartPose start_pose_;
  SE3d base_to_livox_;
  SE3d base_to_depth_;
  SE3d world_livox0_;
  danger_search_localization::PlanarGicpOdometry livox_gicp_;
  danger_search_localization::PlanarGicpOdometry depth_gicp_;
  Eigen::Matrix3d initial_world_base_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d initial_world_base_translation_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d initial_imu_rotation_ = Eigen::Matrix3d::Identity();
  danger_search_localization::ImuMotionPredictor imu_motion_predictor_;
  danger_search_localization::StanceInitializationGate
      stance_initialization_gate_;
  danger_search_localization::StanceHealthMonitor stance_health_monitor_;
  danger_search_localization::PoseVelocityEstimator velocity_estimator_;
  Eigen::Vector3d scan_translation_correction_ = Eigen::Vector3d::Zero();
  bool world_frame_initialized_ = false;
  bool stance_health_enabled_ = true;
  bool use_imu_constrained_ = false;
  bool use_livox_imu_local_map_ = false;
  bool use_adaptive_shared_map_ = false;
  bool use_rgbd_livox_ = false;
  bool imu_anchor_initialized_ = false;
  bool fusion_initialized_ = false;
  bool adaptive_pose_initialized_ = false;
  double min_range_ = 0.5;
  double max_range_ = 15.0;
  double imu_max_delta_ = 0.01;
  double command_velocity_scale_ = 1.0;
  double livox_imu_maximum_speed_ = 0.70;
  double livox_imu_translation_margin_ = 0.02;
  double livox_imu_absolute_translation_cap_ = 0.30;
  danger_search_localization::AdaptiveTranslationConfig
      adaptive_selector_config_;
  double diagnostic_large_translation_threshold_ = 0.08;
  double diagnostic_summary_period_ = 2.0;
  bool diagnostic_frame_details_ = false;
  double depth_freshness_seconds_ = 0.35;
  PlanarRegistrationDiagnosticStats depth_registration_diagnostics_;
  PlanarRegistrationDiagnosticStats livox_registration_diagnostics_;
  MapRegistrationDiagnosticStats map_registration_diagnostics_;
  MotionConsistencyDiagnosticState motion_consistency_diagnostics_;
  bool depth_freshness_state_initialized_ = false;
  bool previous_depth_fresh_ = false;
  ros::Time depth_stale_since_;
  std::deque<ImuSample> imu_samples_;
  mutable std::mutex imu_mutex_;
  mutable std::mutex command_mutex_;
  std::mutex processing_mutex_;
  std::mutex rgbd_scan_mutex_;
  std::mutex shared_map_mutex_;
  std::mutex depth_processing_mutex_;
  std::mutex initialization_mutex_;
  std::mutex fusion_mutex_;
  ros::Time last_depth_process_stamp_;
  ros::Time last_depth_registration_stamp_;
  ros::Time last_livox_registration_stamp_;
  ros::Time last_local_map_accept_stamp_;
  ros::Time last_adaptive_output_stamp_;
  ros::Time adaptive_depth_proposal_stamp_;
  std::deque<CommandSample> command_samples_;
  Eigen::Vector2d depth_total_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d livox_total_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d fused_planar_position_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d fused_at_last_depth_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d livox_at_last_depth_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d depth_fusion_offset_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d map_correction_total_ = Eigen::Vector2d::Zero();
  Eigen::Vector3d local_map_base_translation_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d adaptive_base_translation_ = Eigen::Vector3d::Zero();
  danger_search_localization::TranslationProposal adaptive_depth_proposal_;
  std::uint64_t adaptive_rgbd_selected_count_ = 0;
  std::uint64_t adaptive_livox_selected_count_ = 0;
  std::uint64_t adaptive_both_invalid_count_ = 0;
  Eigen::Vector2d depth_command_total_ = Eigen::Vector2d::Zero();
  ros::Time last_depth_accept_stamp_;
  bool scale_window_initialized_ = false;
  ros::Time scale_window_stamp_;
  Eigen::Vector2d scale_depth_anchor_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d scale_livox_anchor_ = Eigen::Vector2d::Zero();
  std::deque<double> livox_scale_samples_;
  double livox_scale_ = 1.0;
  std::string frame_id_;
  std::string raw_cloud_frame_;
  std::string imu_frame_;
  std::string depth_cloud_frame_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_localization");
  try {
    ScanLocalizationNode node;
    // 点云配准可能占用几十毫秒；IMU 回调必须能并行补充时间缓存，否则
    // 单线程 spinner 会把合法、时间匹配的传感器帧误判为缺少 IMU。
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception &error) {
    // 节点在 ROS 日志连接完成前退出时，ROS_FATAL 可能来不及落盘；
    // 同步写入标准错误，确保正式 runner 能归档真实启动原因。
    std::cerr << "scan_localization 无法启动：" << error.what() << std::endl;
    ROS_FATAL("scan_localization 无法启动：%s", error.what());
    return 1;
  }
  return 0;
}
