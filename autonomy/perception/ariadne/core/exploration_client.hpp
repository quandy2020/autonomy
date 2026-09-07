/*
 * Copyright 2026 The Openbot Authors
 *
 * Autolink I/O for RGB-D exploration: subscribes odom/depth, publishes path.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/std_msgs/bool.pb.h>
#include <automsgs/msgs/std_msgs/float32.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autolink/autolink.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/perception/exploration/common/types.hpp"
#include "autonomy/perception/exploration/core/exploration_options.hpp"
#include "autonomy/perception/exploration/core/explorer.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy::perception::exploration {

// Bridges ExplorerInterface to autolink topics and TF.
class ExplorationClient {
 public:
  struct Options {
    std::string odom_topic = "/odom";
    std::string depth_topic = "/camera/depth/image_raw";
    std::string camera_info_topic = "/camera/depth/camera_info";
    std::string point_cloud_topic = "/velodyne_points";
    std::string camera_frame = "camera_depth_optical_frame";
    std::string map_frame = "map";
    std::string path_topic = kExplorationPathTopic;
    std::string waypoint_topic = kExplorationWaypointTopic;
    std::string map_topic = kExplorationMapTopic;
    std::string global_path_topic = kExplorationGlobalPathTopic;
    std::string local_path_topic = kExplorationLocalPathTopic;
    std::string exploration_finished_topic = kExplorationFinishedTopic;
    std::string exploration_progress_topic = kExplorationProgressTopic;
    std::string navigation_boundary_topic = kNavigationBoundaryTopic;
    std::string terrain_map_topic = "/terrain_map";
    std::string prior_map_topic = "/map";
    std::string planner_costmap_topic = "/global_costmap";
    std::string vg_markers_topic = kExplorationVgMarkersTopic;
    std::string reset_topic = kExplorationResetTopic;
    std::string pause_topic = kExplorationPauseTopic;
    std::string waypoint_reached_topic = kExplorationWaypointReachedTopic;
    bool lidar_primary{false};
    double planner_hz = 2.0;
    std::string config_directory;
    std::string exploration_config = "perception/exploration_rgbd_tare.lua";
    std::string explorer_backend;
  };

  ExplorationClient();
  ~ExplorationClient();

  void SetOptions(const Options& options);
  void SetTransformBuffer(std::shared_ptr<transform::Buffer> tf_buffer);

  bool AttachNode(const std::shared_ptr<autolink::Node>& node);
  void Start();
  void Shutdown();

  ExplorerInterface* explorer() { return explorer_.get(); }
  const ExplorerInterface* explorer() const { return explorer_.get(); }

 private:
  void OnOdometry(const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);
  void OnDepth(const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg);
  void OnPointCloud(
      const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg);
  void OnTerrainMap(
      const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg);
  void OnPriorMap(
      const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& msg);
  void OnPlannerCostmap(
      const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& msg);
  void OnReset(const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& msg);
  void OnPause(const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& msg);
  void OnWaypointReached(
      const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& msg);
  void OnCameraInfo(
      const std::shared_ptr<automsgs::msgs::sensor_msgs::CameraInfo>& msg);
  void PlannerLoop();
  bool LookupMapTCamera(
      automsgs::msgs::geometry_msgs::Transform* map_t_camera) const;
  void PublishOutputs();

  Options options_;
  proto::ExplorationOptions exploration_options_;
  std::unique_ptr<ExplorerInterface> explorer_;

  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<transform::Buffer> tf_buffer_;

  std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>
      odom_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::Image>>
      depth_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::CameraInfo>>
      camera_info_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::PointCloud2>>
      point_cloud_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::PointCloud2>>
      terrain_map_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::map_msgs::OccupancyGrid>>
      prior_map_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::map_msgs::OccupancyGrid>>
      planner_costmap_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::std_msgs::Bool>>
      reset_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::std_msgs::Bool>>
      pause_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::std_msgs::Bool>>
      waypoint_reached_reader_;

  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
      path_writer_;
  std::shared_ptr<
      autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>>
      waypoint_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
      map_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
      global_path_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
      local_path_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::std_msgs::Bool>>
      finished_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::std_msgs::Float32>>
      progress_writer_;
  std::shared_ptr<
      autolink::Writer<automsgs::msgs::geometry_msgs::PolygonStamped>>
      boundary_writer_;
  std::shared_ptr<
      autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
      vg_markers_writer_;

  std::shared_ptr<automsgs::msgs::sensor_msgs::CameraInfo> camera_info_;
  std::mutex camera_info_mutex_;
  bool last_finished_{false};
  bool boundary_published_{false};
  bool has_last_waypoint_{false};
  double last_waypoint_x_{0.0};
  double last_waypoint_y_{0.0};

  std::thread planner_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace autonomy::perception::exploration
