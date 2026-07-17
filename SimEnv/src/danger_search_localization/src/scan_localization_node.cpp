// 参赛方定位节点：只读取初始接口文档授权的 Livox、RGB-D 与机身 IMU。
#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <Eigen/Geometry>
#include <cmath>
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
#include <danger_search_localization/ground_plane_initializer.hpp>
#include <danger_search_localization/imu_constrained_odometry.hpp>
#include <danger_search_localization/imu_motion_predictor.hpp>
#include <danger_search_localization/planar_gicp_odometry.hpp>
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

double WrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

SO3d RotationFromRpy(const std::vector<double> &rpy) {
  if (rpy.size() != 3) {
    throw std::runtime_error("base_to_livox_rpy 必须包含 3 个值");
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
  return {values[0], values[1], values[2]};
}

struct StartPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  bool relative = false;
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
    std::string depth_cloud_topic = "/real_sense/depth/points";
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
    private_nh_.param("depth_cloud_topic", depth_cloud_topic,
                      depth_cloud_topic);
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
    if (localization_backend != "kiss_icp" &&
        localization_backend != "imu_constrained" &&
        localization_backend != "rgbd_livox") {
      throw std::runtime_error(
          "localization_backend 仅支持 kiss_icp、imu_constrained 或 rgbd_livox");
    }
    if (!private_nh_.getParam("base_to_livox_xyz", base_to_livox_xyz) ||
        !private_nh_.getParam("base_to_livox_rpy", base_to_livox_rpy)) {
      throw std::runtime_error("缺少已公开的 base_to_livox 固定外参");
    }
    if (raw_cloud_topic != "/scan") {
      throw std::runtime_error("正式定位仅允许使用初始文档授权的 /scan");
    }
    use_imu_constrained_ = localization_backend == "imu_constrained";
    use_rgbd_livox_ = localization_backend == "rgbd_livox";
    const bool requires_imu = use_imu_constrained_ || use_rgbd_livox_;
    if (use_rgbd_livox_ && depth_cloud_topic != "/real_sense/depth/points") {
      throw std::runtime_error(
          "RGB-D/Livox 定位仅允许使用 /real_sense/depth/points");
    }
    const StartPose start = ReadPublicStartPose(
        scene_info_path, require_scene_info, allow_relative_origin,
        requires_imu, use_rgbd_livox_, use_rgbd_livox_);
    if (requires_imu && start.relative) {
      throw std::runtime_error("IMU 融合定位必须使用公开世界系起点");
    }
    start_pose_ = start;
    base_to_livox_ = SE3d(RotationFromRpy(base_to_livox_rpy),
                           VectorFromParam(base_to_livox_xyz, "base_to_livox_xyz"));
    if (use_rgbd_livox_) {
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
    if (use_imu_constrained_) {
      imu_odometry_ = std::make_unique<
          danger_search_localization::ImuConstrainedOdometry>();
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
    subscriber_ = nh_.subscribe(raw_cloud_topic, 2,
                                &ScanLocalizationNode::OnCloud, this);
    if (requires_imu) {
      imu_subscriber_ = nh_.subscribe(
          "/trunk_imu", 200, &ScanLocalizationNode::OnImu, this);
    }
    if (use_rgbd_livox_) {
      depth_subscriber_ = nh_.subscribe(
          depth_cloud_topic, 1, &ScanLocalizationNode::OnDepthCloud, this);
      command_subscriber_ = nh_.subscribe(
          "/cmd_vel", 50, &ScanLocalizationNode::OnCommand, this);
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
    const ros::Time oldest = stamp - ros::Duration(5.0);
    while (!command_samples_.empty() &&
           command_samples_.front().stamp < oldest) {
      command_samples_.pop_front();
    }
  }

  Eigen::Vector2d CommandMotionPrior(
      const ros::Time &start, const ros::Time &end,
      const Eigen::Matrix3d &world_base_rotation) const {
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
      const double dt = (sample.stamp - cursor).toSec();
      if (dt > 0.0) {
        const Eigen::Vector3d world_velocity =
            world_base_rotation *
            Eigen::Vector3d(current_velocity.x(), current_velocity.y(), 0.0);
        result += world_velocity.head<2>() * dt;
      }
      cursor = sample.stamp;
      current_velocity = sample.base_velocity;
    }
    const double final_dt = (end - cursor).toSec();
    if (final_dt > 0.0) {
      const Eigen::Vector3d world_velocity =
          world_base_rotation *
          Eigen::Vector3d(current_velocity.x(), current_velocity.y(), 0.0);
      result += world_velocity.head<2>() * final_dt;
    }
    return command_velocity_scale_ * result;
  }

  void OnImu(const sensor_msgs::Imu::ConstPtr &message) {
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
      const ros::Time oldest = message->header.stamp - ros::Duration(2.0);
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
    std::lock_guard<std::mutex> lock(imu_mutex_);
    const ImuSample *closest = nullptr;
    double closest_delta = std::numeric_limits<double>::infinity();
    for (const auto &sample : imu_samples_) {
      const double delta = std::abs((sample.stamp - stamp).toSec());
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
    if (!scale_window_initialized_) {
      scale_depth_anchor_ = depth_total_;
      scale_livox_anchor_ = livox_total_;
      scale_window_stamp_ = stamp;
      scale_window_initialized_ = true;
      return;
    }
    if ((stamp - scale_window_stamp_).toSec() < 1.0) {
      return;
    }
    const double depth_distance =
        (depth_total_ - scale_depth_anchor_).norm();
    const double livox_distance =
        (livox_total_ - scale_livox_anchor_).norm();
    if (depth_distance >= 0.20 && livox_distance >= 0.10) {
      const double ratio = depth_distance / livox_distance;
      if (std::isfinite(ratio) && ratio >= 0.70 && ratio <= 2.20) {
        livox_scale_samples_.push_back(ratio);
        while (livox_scale_samples_.size() > 5) {
          livox_scale_samples_.pop_front();
        }
        livox_scale_ = Median(std::vector<double>(
            livox_scale_samples_.begin(), livox_scale_samples_.end()));
      }
    }
    scale_depth_anchor_ = depth_total_;
    scale_livox_anchor_ = livox_total_;
    scale_window_stamp_ = stamp;
  }

  void OnDepthCloud(const sensor_msgs::PointCloud2::ConstPtr &message) {
    if (!use_rgbd_livox_ || message->header.stamp.isZero()) {
      return;
    }
    std::lock_guard<std::mutex> depth_lock(depth_processing_mutex_);
    Eigen::Matrix3d initial_world_base_rotation;
    Eigen::Matrix3d initial_imu_rotation;
    {
      std::lock_guard<std::mutex> state_lock(initialization_mutex_);
      if (!world_frame_initialized_ || !fusion_initialized_) {
        return;
      }
      initial_world_base_rotation = initial_world_base_rotation_;
      initial_imu_rotation = initial_imu_rotation_;
    }
    if (!last_depth_process_stamp_.isZero() &&
        (message->header.stamp - last_depth_process_stamp_).toSec() < 0.18) {
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
    const Eigen::Vector2d command_prior = CommandMotionPrior(
        last_depth_registration_stamp_, message->header.stamp,
        world_base_rotation);
    const auto result = depth_gicp_.RegisterFrame(points, command_prior);
    if (result.point_count >= 80) {
      last_depth_registration_stamp_ = message->header.stamp;
    }
    if (!result.accepted) {
      ROS_WARN_THROTTLE(
          2.0, "RGB-D 平移配准不可用：points=%zu fitness=%.4f",
          result.point_count, result.fitness);
      return;
    }
    {
      std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
      if (result.has_delta) {
        depth_total_ += result.delta;
      }
      const bool recovering =
          last_depth_accept_stamp_.isZero() ||
          (message->header.stamp - last_depth_accept_stamp_).toSec() >
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

    std::unique_lock<std::mutex> process_lock(
        use_rgbd_livox_ ? rgbd_scan_mutex_ : processing_mutex_);

    std::optional<ImuSample> current_imu_sample;
    if (use_imu_constrained_ || use_rgbd_livox_) {
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

    {
      std::lock_guard<std::mutex> initialization_lock(initialization_mutex_);
      if (!world_frame_initialized_) {
      std::vector<Eigen::Vector3d> base_points;
      base_points.reserve(points.size());
      for (const auto &point : points) {
        base_points.push_back(base_to_livox_ * point);
      }
      const auto ground = danger_search_localization::EstimateGroundPlane(base_points);
      if (!ground.has_value()) {
        ROS_WARN_THROTTLE(
            2.0,
            "首帧楼面证据不足，等待下一份 /scan；不会使用出生高度发布错误世界坐标");
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
      if (use_imu_constrained_ || use_rgbd_livox_) {
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
    if (use_rgbd_livox_) {
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
      const auto result = livox_gicp_.RegisterFrame(
          oriented_points, command_prior);
      if (result.point_count >= 80) {
        last_livox_registration_stamp_ = message->header.stamp;
      }
      bool depth_fresh = false;
      Eigen::Vector2d fused_position;
      Eigen::Vector2d depth_total;
      Eigen::Vector2d livox_total;
      double livox_scale = 1.0;
      {
        std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
        if (result.accepted && result.has_delta) {
          livox_total_ += result.delta;
        }
        depth_fresh =
            !last_depth_accept_stamp_.isZero() &&
            std::abs((message->header.stamp - last_depth_accept_stamp_).toSec()) <=
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
      }
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
    PublishOdometry(message->header.stamp, world_base);
    PublishRegisteredCloud(message->header.stamp, world_livox, points);

    std_msgs::Bool ready;
    ready.data = true;
    ready_publisher_.publish(ready);
  }

  void PublishOdometry(const ros::Time &stamp, const SE3d &world_base) {
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

    if (!previous_stamp_.isZero() && stamp > previous_stamp_) {
      const double dt = (stamp - previous_stamp_).toSec();
      const Eigen::Vector3d velocity_world = (translation - previous_translation_) / dt;
      // 里程计 twist 在 child_frame_id="base" 下表达，需将世界系速度转换到 base 系。
      const Eigen::Vector3d velocity_base =
          danger_search_localization::WorldVelocityToBase(
              velocity_world, world_base.rotationMatrix());
      message.twist.twist.linear.x = velocity_base.x();
      message.twist.twist.linear.y = velocity_base.y();
      message.twist.twist.linear.z = velocity_base.z();
      message.twist.twist.angular.z = WrapAngle(
          YawFromRotation(world_base.rotationMatrix()) - previous_yaw_) / dt;
    }
    previous_stamp_ = stamp;
    previous_translation_ = translation;
    previous_yaw_ = YawFromRotation(world_base.rotationMatrix());
    odom_publisher_.publish(message);
  }

  void PublishRegisteredCloud(const ros::Time &stamp, const SE3d &world_livox,
                              const std::vector<Eigen::Vector3d> &points) {
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

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber depth_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Publisher odom_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Publisher ready_publisher_;
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
  Eigen::Vector3d scan_translation_correction_ = Eigen::Vector3d::Zero();
  bool world_frame_initialized_ = false;
  bool use_imu_constrained_ = false;
  bool use_rgbd_livox_ = false;
  bool imu_anchor_initialized_ = false;
  bool fusion_initialized_ = false;
  double min_range_ = 0.5;
  double max_range_ = 15.0;
  double imu_max_delta_ = 0.01;
  double command_velocity_scale_ = 1.0;
  double depth_freshness_seconds_ = 0.35;
  std::deque<ImuSample> imu_samples_;
  mutable std::mutex imu_mutex_;
  mutable std::mutex command_mutex_;
  std::mutex processing_mutex_;
  std::mutex rgbd_scan_mutex_;
  std::mutex depth_processing_mutex_;
  std::mutex initialization_mutex_;
  std::mutex fusion_mutex_;
  ros::Time last_depth_process_stamp_;
  ros::Time last_depth_registration_stamp_;
  ros::Time last_livox_registration_stamp_;
  std::deque<CommandSample> command_samples_;
  Eigen::Vector2d depth_total_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d livox_total_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d fused_planar_position_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d fused_at_last_depth_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d livox_at_last_depth_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d depth_fusion_offset_ = Eigen::Vector2d::Zero();
  ros::Time last_depth_accept_stamp_;
  bool scale_window_initialized_ = false;
  ros::Time scale_window_stamp_;
  Eigen::Vector2d scale_depth_anchor_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d scale_livox_anchor_ = Eigen::Vector2d::Zero();
  std::deque<double> livox_scale_samples_;
  double livox_scale_ = 1.0;
  std::string frame_id_;
  ros::Time previous_stamp_;
  Eigen::Vector3d previous_translation_ = Eigen::Vector3d::Zero();
  double previous_yaw_ = 0.0;
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
