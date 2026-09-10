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
    AWARN << "PerceptionServer: exploration backend is not built.";
  }

  running_ = true;
}

void PerceptionServer::Shutdown() {
  if (!running_) {
    return;
  }
  node_.reset();
  running_ = false;
}

}  // namespace perception
}  // namespace autonomy
