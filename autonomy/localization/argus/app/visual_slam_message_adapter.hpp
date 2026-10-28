/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#ifndef ARGUS_APP_VISUAL_SLAM_MESSAGE_ADAPTER_HPP_
#define ARGUS_APP_VISUAL_SLAM_MESSAGE_ADAPTER_HPP_

#include <cstddef>
#include <mutex>
#include <string>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "visual_slam_system.hpp"

namespace argus {
namespace app {

class VisualSlamMessageAdapter {
 public:
  struct Options {
    std::string map_frame = "map";
    std::string camera_frame = "camera";
    std::size_t maximum_path_poses = 10000;
  };

  explicit VisualSlamMessageAdapter(VisualSlamSystem* system);
  VisualSlamMessageAdapter(VisualSlamSystem* system, Options options);

  bool AddImu(const ::automsgs::msgs::sensor_msgs::Imu& message);
  bool TrackMonocular(
      const ::automsgs::msgs::sensor_msgs::Image& image);
  bool TrackStereo(
      const ::automsgs::msgs::sensor_msgs::Image& left,
      const ::automsgs::msgs::sensor_msgs::Image& right);
  bool TrackRgbd(
      const ::automsgs::msgs::sensor_msgs::Image& color,
      const ::automsgs::msgs::sensor_msgs::Image& depth);

  bool GetOdometry(
      ::automsgs::msgs::nav_msgs::Odometry* message) const;
  bool GetPath(::automsgs::msgs::nav_msgs::Path* message) const;
  bool GetTrackedMap(
      ::automsgs::msgs::sensor_msgs::PointCloud2* message) const;

 private:
  bool Record(const VisualTrackingResult& result);

  VisualSlamSystem* system_ = nullptr;
  Options options_;
  mutable std::mutex mutex_;
  ::automsgs::msgs::nav_msgs::Path path_;
  VisualTrackingResult latest_;
};

}  // namespace app
}  // namespace argus

#endif  // ARGUS_APP_VISUAL_SLAM_MESSAGE_ADAPTER_HPP_
