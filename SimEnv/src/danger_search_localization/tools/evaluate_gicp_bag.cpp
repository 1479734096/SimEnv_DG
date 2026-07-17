// 离线比较 GICP 与 Gazebo 标签；真值只参与配准结果生成后的误差计算。
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Eigen::Matrix3d Rotation(const geometry_msgs::Quaternion &message) {
  Eigen::Quaterniond quaternion(message.w, message.x, message.y, message.z);
  quaternion.normalize();
  return quaternion.toRotationMatrix();
}

Cloud::Ptr OrientedCloud(const sensor_msgs::PointCloud &message,
                         const Eigen::Matrix3d &rotation) {
  Cloud::Ptr raw(new Cloud);
  raw->reserve(message.points.size());
  for (const auto &point : message.points) {
    const Eigen::Vector3d candidate(point.x, point.y, point.z);
    const double range = candidate.norm();
    if (!candidate.allFinite() || range < 0.5 || range > 15.0) {
      continue;
    }
    const Eigen::Vector3d oriented = rotation * candidate;
    raw->emplace_back(
        static_cast<float>(oriented.x()), static_cast<float>(oriented.y()),
        static_cast<float>(oriented.z()));
  }
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(0.18f, 0.18f, 0.18f);
  voxel.setInputCloud(raw);
  voxel.filter(*filtered);
  return filtered;
}

Cloud::Ptr OrientedDepthCloud(const sensor_msgs::PointCloud2 &message,
                              const Eigen::Matrix3d &rotation) {
  Cloud::Ptr raw(new Cloud);
  raw->reserve(message.width * message.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    const Eigen::Vector3d candidate(*x, *y, *z);
    const double range = candidate.norm();
    if (!candidate.allFinite() || range < 0.20 || range > 8.0) {
      continue;
    }
    const Eigen::Vector3d oriented = rotation * candidate;
    raw->emplace_back(
        static_cast<float>(oriented.x()), static_cast<float>(oriented.y()),
        static_cast<float>(oriented.z()));
  }
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(0.08f, 0.08f, 0.08f);
  voxel.setInputCloud(raw);
  voxel.filter(*filtered);
  return filtered;
}

double Quantile(std::vector<double> values, double fraction) {
  const std::size_t index = static_cast<std::size_t>(
      std::floor(fraction * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "用法：evaluate_gicp_bag <bag> [--depth]" << std::endl;
    return 2;
  }
  const bool use_depth = argc == 3 && std::string(argv[2]) == "--depth";
  const std::string cloud_topic =
      use_depth ? "/real_sense/depth/points" : "/scan";
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  const std::vector<std::string> topics{
      cloud_topic, "/trunk_imu", "/ground_truth/base_w"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  Eigen::Matrix3d latest_imu = Eigen::Matrix3d::Identity();
  bool have_imu = false;
  Eigen::Vector3d latest_truth = Eigen::Vector3d::Zero();
  bool have_truth = false;
  Cloud::Ptr previous;
  Eigen::Vector3d estimated = Eigen::Vector3d::Zero();
  Eigen::Vector3d truth_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d truth_last = Eigen::Vector3d::Zero();
  std::vector<double> errors;
  std::vector<double> fitness;
  std::size_t accepted = 0;
  std::size_t rejected = 0;
  std::size_t cloud_index = 0;
  double last_report_stamp = -std::numeric_limits<double>::infinity();
  const Eigen::Matrix3d base_to_optical_rotation =
      (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

  BOOST_FOREACH (const rosbag::MessageInstance &instance, view) {
    if (instance.getTopic() == "/trunk_imu") {
      const auto message = instance.instantiate<sensor_msgs::Imu>();
      if (message) {
        latest_imu = Rotation(message->orientation);
        have_imu = true;
      }
      continue;
    }
    if (instance.getTopic() == "/ground_truth/base_w") {
      const auto message = instance.instantiate<nav_msgs::Odometry>();
      if (message) {
        latest_truth = Eigen::Vector3d(
            message->pose.pose.position.x, message->pose.pose.position.y,
            message->pose.pose.position.z);
        have_truth = true;
      }
      continue;
    }
    if (!have_imu || !have_truth) {
      continue;
    }
    ++cloud_index;
    if (use_depth && cloud_index % 2 != 0) {
      continue;
    }
    Cloud::Ptr current;
    double cloud_stamp = 0.0;
    if (use_depth) {
      const auto message = instance.instantiate<sensor_msgs::PointCloud2>();
      if (!message) {
        continue;
      }
      cloud_stamp = message->header.stamp.toSec();
      current = OrientedDepthCloud(
          *message, latest_imu * base_to_optical_rotation);
    } else {
      const auto message = instance.instantiate<sensor_msgs::PointCloud>();
      if (!message) {
        continue;
      }
      cloud_stamp = message->header.stamp.toSec();
      current = OrientedCloud(*message, latest_imu);
    }
    if (!current || current->size() < 50) {
      ++rejected;
      continue;
    }
    if (!previous) {
      previous = current;
      truth_start = latest_truth;
      truth_last = latest_truth;
      continue;
    }

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(35);
    gicp.setMaxCorrespondenceDistance(use_depth ? 0.25 : 0.55);
    gicp.setTransformationEpsilon(1e-4);
    gicp.setEuclideanFitnessEpsilon(1e-4);
    gicp.setInputSource(current);
    gicp.setInputTarget(previous);
    Cloud aligned;
    gicp.align(aligned);
    const Eigen::Matrix4f relative = gicp.getFinalTransformation();
    Eigen::Vector3d frame_delta =
        relative.block<3, 1>(0, 3).cast<double>();
    // IMU 已提供世界系姿态；首版只评价平面平移，避免 GICP 的退化 z 分量
    // 被误当成楼层高度。
    frame_delta.z() = 0.0;
    const double score = gicp.getFitnessScore(use_depth ? 0.25 : 0.55);
    const bool valid = gicp.hasConverged() && std::isfinite(score) &&
                       frame_delta.head<2>().norm() <= 0.30 &&
                       std::abs(frame_delta.z()) <= 0.20;
    if (valid) {
      estimated += frame_delta;
      truth_last = latest_truth;
      const Eigen::Vector3d truth_delta = latest_truth - truth_start;
      errors.push_back((estimated - truth_delta).head<2>().norm());
      fitness.push_back(score);
      ++accepted;
      if (cloud_stamp - last_report_stamp >= 2.0) {
        std::cout << "sample stamp=" << cloud_stamp
                  << " truth_xy=" << truth_delta.head<2>().transpose()
                  << " estimated_xy=" << estimated.head<2>().transpose()
                  << " error=" << errors.back() << '\n';
        last_report_stamp = cloud_stamp;
      }
    } else {
      ++rejected;
    }
    previous = current;
  }
  bag.close();
  if (errors.empty()) {
    std::cerr << "没有 GICP 帧通过质量门槛" << std::endl;
    return 1;
  }
  const Eigen::Vector3d truth_displacement = truth_last - truth_start;
  std::cout << "source=" << cloud_topic << '\n';
  std::cout << "accepted=" << accepted << " rejected=" << rejected
            << " acceptance="
            << static_cast<double>(accepted) /
                   static_cast<double>(accepted + rejected)
            << '\n';
  std::cout << "error_median=" << Quantile(errors, 0.5)
            << " error_p95=" << Quantile(errors, 0.95)
            << " error_max=" << *std::max_element(errors.begin(), errors.end())
            << '\n';
  std::cout << "fitness_median=" << Quantile(fitness, 0.5) << '\n';
  std::cout << "truth_delta=" << truth_displacement.transpose()
            << " estimated_delta=" << estimated.transpose() << std::endl;
  return 0;
}
