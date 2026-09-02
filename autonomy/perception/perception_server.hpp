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

#pragma once

#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/perception/exploration/core/exploration_client.hpp"
#include "autonomy/perception/proto/perception_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace perception {

constexpr char kPerceptionServerNodeName[] = "perception_server";

// Process-level facade for perception backends (RGB-D exploration, future vision).
class PerceptionServer {
 public:
  AUTONOMY_SMART_PTR_DEFINITIONS(PerceptionServer)

  explicit PerceptionServer(const proto::PerceptionOptions& options);
  ~PerceptionServer();

  PerceptionServer(const PerceptionServer&) = delete;
  PerceptionServer& operator=(const PerceptionServer&) = delete;

  void SetConfigDirectory(const std::string& config_directory);
  void SetTransformBuffer(std::shared_ptr<transform::Buffer> tf_buffer);

  void Start();
  void Shutdown();

  [[nodiscard]] bool is_running() const { return running_; }
  [[nodiscard]] const proto::PerceptionOptions& options() const {
    return options_;
  }

  exploration::ExplorationClient* exploration_client() {
    return exploration_client_.get();
  }

 private:
  proto::PerceptionOptions options_;
  std::string config_directory_;
  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<transform::Buffer> tf_buffer_;
  std::unique_ptr<exploration::ExplorationClient> exploration_client_;
  bool running_{false};
};

}  // namespace perception
}  // namespace autonomy
