/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/core/exploration_client.hpp"

#include <chrono>
#include <cmath>

#include "autonomy/common/logging.hpp"

namespace autonomy::perception::exploration {
namespace {

using automsgs::msgs::geometry_msgs::PolygonStamped;
using automsgs::msgs::geometry_msgs::PoseStamped;
using automsgs::msgs::geometry_msgs::Transform;
using automsgs::msgs::map_msgs::OccupancyGrid;
using automsgs::msgs::nav_msgs::Odometry;
using automsgs::msgs::nav_msgs::Path;
using automsgs::msgs::sensor_msgs::CameraInfo;
using automsgs::msgs::sensor_msgs::Image;
using automsgs::msgs::sensor_msgs::PointCloud2;
using automsgs::msgs::std_msgs::Bool;
using automsgs::msgs::std_msgs::Float32;

std::string ResolveBackend(
    const ExplorationClient::Options& options,
    const proto::ExplorationOptions& exploration) {
  if (!options.explorer_backend.empty()) {
    return options.explorer_backend;
  }
  if (!exploration.explorer_backend().empty()) {
    return exploration.explorer_backend();
  }
  return "rgbd_tare";
}

}  // namespace

ExplorationClient::ExplorationClient() {
  exploration_options_ = DefaultOptions();
  explorer_ = ExplorerFactory::Create(ResolveBackend(options_, exploration_options_),
                                    exploration_options_);
}

ExplorationClient::~ExplorationClient() { Shutdown(); }

void ExplorationClient::SetOptions(const Options& options) {
  options_ = options;
  if (!options_.config_directory.empty()) {
    exploration_options_ = LoadOptions(options_.config_directory,
                                       options_.exploration_config);
  } else {
    exploration_options_ = DefaultOptions();
  }
  if (!options_.map_frame.empty()) {
    exploration_options_.set_map_frame(options_.map_frame);
  }
  options_.lidar_primary =
      ResolveBackend(options_, exploration_options_) == "lidar_tare" ||
      exploration_options_.use_lidar_primary();
  const std::string backend =
      ResolveBackend(options_, exploration_options_);
  explorer_ = ExplorerFactory::Create(backend, exploration_options_);
  explorer_->Configure(exploration_options_);
  if (!exploration_options_.prior_map_topic().empty()) {
    options_.prior_map_topic = exploration_options_.prior_map_topic();
  }
  if (!exploration_options_.planner_costmap_topic().empty()) {
    options_.planner_costmap_topic =
        exploration_options_.planner_costmap_topic();
  }
  if (!options_.config_directory.empty()) {
    explorer_->LoadBoundaries({options_.config_directory});
  }
}

void ExplorationClient::SetTransformBuffer(
    std::shared_ptr<transform::Buffer> tf_buffer) {
  tf_buffer_ = std::move(tf_buffer);
}

bool ExplorationClient::AttachNode(const std::shared_ptr<autolink::Node>& node) {
  node_ = node;
  return static_cast<bool>(node_);
}

void ExplorationClient::Start() {
  if (running_.exchange(true)) {
    return;
  }
  if (!node_ || !explorer_) {
    AERROR << "ExplorationClient: AttachNode() before Start()";
    running_ = false;
    return;
  }

  ExplorationClient* self = this;
  if (!options_.odom_topic.empty()) {
    odom_reader_ = node_->CreateReader<Odometry>(
        options_.odom_topic,
        [self](const std::shared_ptr<Odometry>& msg) { self->OnOdometry(msg); });
  }
  if (!options_.depth_topic.empty() && !options_.lidar_primary) {
    depth_reader_ = node_->CreateReader<Image>(
        options_.depth_topic,
        [self](const std::shared_ptr<Image>& msg) { self->OnDepth(msg); });
  }
  if (!options_.camera_info_topic.empty() && !options_.lidar_primary) {
    camera_info_reader_ = node_->CreateReader<CameraInfo>(
        options_.camera_info_topic,
        [self](const std::shared_ptr<CameraInfo>& msg) {
          self->OnCameraInfo(msg);
        });
  }
  if (!options_.point_cloud_topic.empty()) {
    point_cloud_reader_ = node_->CreateReader<PointCloud2>(
        options_.point_cloud_topic,
        [self](const std::shared_ptr<PointCloud2>& msg) {
          self->OnPointCloud(msg);
        });
  }
  if (!options_.terrain_map_topic.empty()) {
    terrain_map_reader_ = node_->CreateReader<PointCloud2>(
        options_.terrain_map_topic,
        [self](const std::shared_ptr<PointCloud2>& msg) {
          self->OnTerrainMap(msg);
        });
  }
  if (exploration_options_.use_prior_map() &&
      !options_.prior_map_topic.empty()) {
    prior_map_reader_ = node_->CreateReader<OccupancyGrid>(
        options_.prior_map_topic,
        [self](const std::shared_ptr<OccupancyGrid>& msg) {
          self->OnPriorMap(msg);
        });
  }
  if (exploration_options_.use_planner_costmap() &&
      !options_.planner_costmap_topic.empty()) {
    planner_costmap_reader_ = node_->CreateReader<OccupancyGrid>(
        options_.planner_costmap_topic,
        [self](const std::shared_ptr<OccupancyGrid>& msg) {
          self->OnPlannerCostmap(msg);
        });
  }
  if (!options_.reset_topic.empty()) {
    reset_reader_ = node_->CreateReader<Bool>(
        options_.reset_topic,
        [self](const std::shared_ptr<Bool>& msg) { self->OnReset(msg); });
  }
  if (!options_.pause_topic.empty()) {
    pause_reader_ = node_->CreateReader<Bool>(
        options_.pause_topic,
        [self](const std::shared_ptr<Bool>& msg) { self->OnPause(msg); });
  }
  if (!options_.waypoint_reached_topic.empty()) {
    waypoint_reached_reader_ = node_->CreateReader<Bool>(
        options_.waypoint_reached_topic,
        [self](const std::shared_ptr<Bool>& msg) {
          self->OnWaypointReached(msg);
        });
  }

  if (!options_.path_topic.empty()) {
    path_writer_ = node_->CreateWriter<Path>(options_.path_topic);
  }
  if (!options_.waypoint_topic.empty()) {
    waypoint_writer_ = node_->CreateWriter<PoseStamped>(options_.waypoint_topic);
  }
  if (!options_.map_topic.empty()) {
    map_writer_ = node_->CreateWriter<OccupancyGrid>(options_.map_topic);
  }
  if (!options_.global_path_topic.empty()) {
    global_path_writer_ = node_->CreateWriter<Path>(options_.global_path_topic);
  }
  if (!options_.local_path_topic.empty()) {
    local_path_writer_ = node_->CreateWriter<Path>(options_.local_path_topic);
  }
  if (!options_.exploration_finished_topic.empty()) {
    finished_writer_ =
        node_->CreateWriter<Bool>(options_.exploration_finished_topic);
  }
  if (!options_.exploration_progress_topic.empty()) {
    progress_writer_ =
        node_->CreateWriter<Float32>(options_.exploration_progress_topic);
  }
  if (!options_.navigation_boundary_topic.empty()) {
    boundary_writer_ = node_->CreateWriter<PolygonStamped>(
        options_.navigation_boundary_topic);
  }
  if (exploration_options_.publish_vg_markers() &&
      !options_.vg_markers_topic.empty()) {
    vg_markers_writer_ = node_->CreateWriter<
        automsgs::msgs::visualization_msgs::MarkerArray>(
        options_.vg_markers_topic);
  }

  const double hz = options_.planner_hz > 0.0 ? options_.planner_hz : 2.0;
  planner_thread_ = std::thread([self, hz]() { self->PlannerLoop(); });
  AINFO << "ExplorationClient started (planner " << hz << " Hz, backend "
        << ResolveBackend(options_, exploration_options_) << ")";
}

void ExplorationClient::Shutdown() {
  if (!running_.exchange(false)) {
    return;
  }
  if (planner_thread_.joinable()) {
    planner_thread_.join();
  }
  odom_reader_.reset();
  depth_reader_.reset();
  camera_info_reader_.reset();
  point_cloud_reader_.reset();
  terrain_map_reader_.reset();
  prior_map_reader_.reset();
  planner_costmap_reader_.reset();
  reset_reader_.reset();
  pause_reader_.reset();
  waypoint_reached_reader_.reset();
  path_writer_.reset();
  waypoint_writer_.reset();
  map_writer_.reset();
  global_path_writer_.reset();
  local_path_writer_.reset();
  finished_writer_.reset();
  progress_writer_.reset();
  boundary_writer_.reset();
  vg_markers_writer_.reset();
  node_.reset();
}

void ExplorationClient::OnOdometry(const std::shared_ptr<Odometry>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  explorer_->UpdateOdometry(*msg);
}

void ExplorationClient::OnCameraInfo(const std::shared_ptr<CameraInfo>& msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  camera_info_ = msg;
}

void ExplorationClient::OnDepth(const std::shared_ptr<Image>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  std::shared_ptr<CameraInfo> info;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    info = camera_info_;
  }
  if (!info) {
    return;
  }
  Transform map_t_camera;
  if (!LookupMapTCamera(&map_t_camera)) {
    return;
  }
  explorer_->UpdateDepth(*msg, *info, map_t_camera);
}

void ExplorationClient::OnPointCloud(
    const std::shared_ptr<PointCloud2>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  explorer_->UpdatePointCloud(*msg);
}

void ExplorationClient::OnTerrainMap(
    const std::shared_ptr<PointCloud2>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  explorer_->UpdateTerrainMap(*msg);
}

void ExplorationClient::OnPriorMap(
    const std::shared_ptr<OccupancyGrid>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  explorer_->UpdatePriorMap(*msg);
}

void ExplorationClient::OnPlannerCostmap(
    const std::shared_ptr<OccupancyGrid>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  explorer_->UpdatePlannerCostmap(*msg);
}

void ExplorationClient::OnReset(const std::shared_ptr<Bool>& msg) {
  if (!msg || !explorer_ || !msg->data()) {
    return;
  }
  explorer_->Reset();
  has_last_waypoint_ = false;
  last_finished_ = false;
  boundary_published_ = false;
}

void ExplorationClient::OnPause(const std::shared_ptr<Bool>& msg) {
  if (!msg || !explorer_) {
    return;
  }
  explorer_->SetPaused(msg->data());
}

void ExplorationClient::OnWaypointReached(
    const std::shared_ptr<Bool>& msg) {
  if (!msg || !explorer_ || !msg->data()) {
    return;
  }
  explorer_->MarkWaypointReached();
  has_last_waypoint_ = false;
}

bool ExplorationClient::LookupMapTCamera(Transform* map_t_camera) const {
  if (!tf_buffer_ || map_t_camera == nullptr) {
    return false;
  }
  try {
    automsgs::msgs::builtin_interfaces::Time time;
    const auto stamped = tf_buffer_->lookupTransform(
        options_.map_frame, options_.camera_frame, time);
    *map_t_camera = stamped.transform();
    return true;
  } catch (const std::exception& ex) {
    AWARN << "ExplorationClient TF lookup failed: " << ex.what();
    return false;
  }
}

void ExplorationClient::PlannerLoop() {
  const double hz = options_.planner_hz > 0.0 ? options_.planner_hz : 2.0;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  while (running_) {
    const auto start = std::chrono::steady_clock::now();
    if (explorer_ && explorer_->ExecutePlanningCycle()) {
      PublishOutputs();
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

void ExplorationClient::PublishOutputs() {
  if (!explorer_) {
    return;
  }
  if (path_writer_) {
    path_writer_->Write(explorer_->GetExplorationPath());
  }
  if (waypoint_writer_ && explorer_->HasTarget()) {
    PoseStamped waypoint;
    if (explorer_->GetNextWaypoint(waypoint)) {
      const double x = waypoint.pose().position().x();
      const double y = waypoint.pose().position().y();
      const bool changed =
          !has_last_waypoint_ ||
          std::hypot(x - last_waypoint_x_, y - last_waypoint_y_) > 0.15;
      if (changed) {
        waypoint_writer_->Write(waypoint);
        has_last_waypoint_ = true;
        last_waypoint_x_ = x;
        last_waypoint_y_ = y;
      }
    }
  }
  if (map_writer_) {
    map_writer_->Write(explorer_->GetOccupancyGrid(options_.map_frame));
  }
  if (global_path_writer_) {
    global_path_writer_->Write(explorer_->GetGlobalDebugPath());
  }
  if (local_path_writer_) {
    local_path_writer_->Write(explorer_->GetLocalDebugPath());
  }
  if (progress_writer_) {
    Float32 progress;
    progress.set_data(explorer_->Progress());
    progress_writer_->Write(progress);
  }
  const bool finished = explorer_->IsFinished();
  if (finished_writer_ && finished != last_finished_) {
    Bool msg;
    msg.set_data(finished);
    finished_writer_->Write(msg);
    last_finished_ = finished;
  }
  if (boundary_writer_ && !boundary_published_) {
    automsgs::msgs::geometry_msgs::Polygon boundary;
    if (explorer_->GetNavigationBoundary(&boundary)) {
      PolygonStamped stamped;
      stamped.mutable_header()->set_frame_id(options_.map_frame);
      *stamped.mutable_polygon() = boundary;
      boundary_writer_->Write(stamped);
      boundary_published_ = true;
    }
  }
  if (vg_markers_writer_ && exploration_options_.publish_vg_markers()) {
    auto markers = explorer_->GetVisibilityGraphMarkers(options_.map_frame);
    if (markers.markers_size() > 0) {
      vg_markers_writer_->Write(markers);
    }
  }
}

}  // namespace autonomy::perception::exploration
