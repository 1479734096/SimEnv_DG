// 参赛方定位节点：只读取初始接口文档授权的 /scan。
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
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

StartPose ReadPublicStartPose(const std::string &path, bool require_info, bool allow_relative) {
  boost::property_tree::ptree tree;
  try {
    if (std::filesystem::path(path).filename() != "team_scene_info.json") {
      throw std::runtime_error("正式定位只能读取 team_scene_info.json");
    }
    boost::property_tree::read_json(path, tree);
    if (tree.get<std::string>("schema") != "team_scene_info_v1") {
      throw std::runtime_error("team_scene_info schema 不是 team_scene_info_v1");
    }
    StartPose pose;
    pose.x = tree.get<double>("robot_start.x");
    pose.y = tree.get<double>("robot_start.y");
    pose.z = tree.get<double>("robot_start.z");
    pose.yaw = tree.get<double>("robot_start.yaw");
    bool scan_allowed = false;
    for (const auto &entry : tree.get_child("allowed_interfaces.topics")) {
      if (entry.second.get_value<std::string>() == "/scan") {
        scan_allowed = true;
        break;
      }
    }
    if (!scan_allowed) {
      throw std::runtime_error("公开场景信息未授权 /scan");
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
    std::string scene_info_path;
    bool require_scene_info = true;
    bool allow_relative_origin = false;
    double min_range = 0.5;
    double max_range = 15.0;
    double voxel_size = 0.15;
    std::vector<double> base_to_livox_xyz;
    std::vector<double> base_to_livox_rpy;

    private_nh_.param("raw_cloud_topic", raw_cloud_topic, raw_cloud_topic);
    private_nh_.param("team_scene_info_path", scene_info_path, std::string());
    private_nh_.param("require_team_scene_info", require_scene_info, require_scene_info);
    private_nh_.param("allow_relative_origin", allow_relative_origin, allow_relative_origin);
    private_nh_.param("min_range", min_range, min_range);
    private_nh_.param("max_range", max_range, max_range);
    private_nh_.param("voxel_size", voxel_size, voxel_size);
    if (!private_nh_.getParam("base_to_livox_xyz", base_to_livox_xyz) ||
        !private_nh_.getParam("base_to_livox_rpy", base_to_livox_rpy)) {
      throw std::runtime_error("缺少已公开的 base_to_livox 固定外参");
    }
    if (raw_cloud_topic != "/scan") {
      throw std::runtime_error("正式定位仅允许使用初始文档授权的 /scan");
    }
    const StartPose start = ReadPublicStartPose(
        scene_info_path, require_scene_info, allow_relative_origin);
    const SE3d world_base0(
        SO3d(Eigen::AngleAxisd(start.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()),
        Eigen::Vector3d(start.x, start.y, start.z));
    base_to_livox_ = SE3d(RotationFromRpy(base_to_livox_rpy),
                           VectorFromParam(base_to_livox_xyz, "base_to_livox_xyz"));
    world_livox0_ = world_base0 * base_to_livox_;
    frame_id_ = start.relative ? "danger_search_relative" : "world";

    kiss_icp::pipeline::KISSConfig config;
    config.min_range = min_range;
    config.max_range = max_range;
    config.voxel_size = voxel_size;
    config.deskew = false;
    odometry_ = kiss_icp::pipeline::KissICP(config);

    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(
        "/danger_search/localization/odom", 4);
    cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/danger_search/localization/cloud_registered", 2);
    ready_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/danger_search/localization/ready", 1, true);
    subscriber_ = nh_.subscribe(raw_cloud_topic, 2,
                                &ScanLocalizationNode::OnCloud, this);
    ROS_INFO("scan_localization 已启动：输入 /scan，deskew 关闭，坐标系=%s",
             frame_id_.c_str());
  }

 private:
  void OnCloud(const sensor_msgs::PointCloud::ConstPtr &message) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(message->points.size());
    for (const auto &point : message->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.emplace_back(point.x, point.y, point.z);
      }
    }
    if (points.size() < 50) {
      ROS_WARN_THROTTLE(2.0, "/scan 有效点不足，跳过定位帧");
      return;
    }

    odometry_.RegisterFrame(points);
    const SE3d local_livox = odometry_.poses().back();
    const SE3d world_livox = world_livox0_ * local_livox;
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
      const Eigen::Vector3d velocity = (translation - previous_translation_) / dt;
      message.twist.twist.linear.x = velocity.x();
      message.twist.twist.linear.y = velocity.y();
      message.twist.twist.linear.z = velocity.z();
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
  ros::Publisher odom_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Publisher ready_publisher_;
  kiss_icp::pipeline::KissICP odometry_;
  SE3d base_to_livox_;
  SE3d world_livox0_;
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
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL("scan_localization 无法启动：%s", error.what());
    return 1;
  }
  return 0;
}
