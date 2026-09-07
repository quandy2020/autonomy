/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/perception/perception_server.hpp"

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace perception {

PerceptionServer::PerceptionServer(const proto::PerceptionOptions& options)
    : options_(options) {}

PerceptionServer::~PerceptionServer() { Shutdown(); }

void PerceptionServer::SetConfigDirectory(const std::string& config_directory) {
  config_directory_ = config_directory;
}

void PerceptionServer::SetTransformBuffer(
    std::shared_ptr<transform::Buffer> tf_buffer) {
  tf_buffer_ = std::move(tf_buffer);
  if (exploration_client_) {
    exploration_client_->SetTransformBuffer(tf_buffer_);
  }
}

void PerceptionServer::Start() {
  if (running_) {
    return;
  }
  if (!options_.enabled()) {
    AINFO << "PerceptionServer disabled in configuration.";
    return;
  }

  node_ = autolink::CreateNode(kPerceptionServerNodeName);
  if (!node_) {
    AERROR << "PerceptionServer: failed to create autolink node.";
    return;
  }

  if (options_.enable_rgbd_exploration()) {
    exploration_client_ = std::make_unique<exploration::ExplorationClient>();
    exploration::ExplorationClient::Options client_options;
    client_options.config_directory = config_directory_;
    client_options.exploration_config = options_.exploration_config();
    client_options.odom_topic = options_.odom_topic();
    client_options.depth_topic = options_.depth_topic();
    client_options.camera_info_topic = options_.camera_info_topic();
    client_options.camera_frame = options_.camera_frame();
    client_options.map_frame = options_.map_frame();
    client_options.planner_hz = options_.planner_hz();
    client_options.path_topic = options_.path_topic();
    client_options.waypoint_topic = options_.waypoint_topic();
    client_options.map_topic = options_.map_topic();
    client_options.explorer_backend = options_.explorer_backend();
    client_options.point_cloud_topic = options_.point_cloud_topic();
    client_options.global_path_topic = options_.global_path_topic();
    client_options.local_path_topic = options_.local_path_topic();
    client_options.exploration_finished_topic =
        options_.exploration_finished_topic();
    client_options.exploration_progress_topic =
        options_.exploration_progress_topic();
    client_options.navigation_boundary_topic =
        options_.navigation_boundary_topic();
    client_options.terrain_map_topic = options_.terrain_map_topic();
    client_options.waypoint_reached_topic =
        exploration::kExplorationWaypointReachedTopic;
    if (!options_.prior_map_topic().empty()) {
      client_options.prior_map_topic = options_.prior_map_topic();
    }
    if (!options_.vg_markers_topic().empty()) {
      client_options.vg_markers_topic = options_.vg_markers_topic();
    }
    if (options_.explorer_backend() == "lidar_tare") {
      client_options.lidar_primary = true;
      client_options.depth_topic.clear();
      client_options.camera_info_topic.clear();
    }
    exploration_client_->SetOptions(client_options);
    exploration_client_->SetTransformBuffer(tf_buffer_);
    exploration_client_->AttachNode(node_);
    exploration_client_->Start();
    AINFO << "PerceptionServer: RGB-D exploration client started.";
  }

  running_ = true;
}

void PerceptionServer::Shutdown() {
  if (!running_) {
    return;
  }
  if (exploration_client_) {
    exploration_client_->Shutdown();
    exploration_client_.reset();
  }
  node_.reset();
  running_ = false;
}

}  // namespace perception
}  // namespace autonomy
