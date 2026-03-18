/*
 * Copyright 2018 The Cartographer Authors
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

#include "autonomy/localization/cartographer/cloud/internal/map_builder_server.hpp"
#include "autonomy/localization/cartographer/mapping/internal/2d/local_slam_result_2d.hpp"
#include "autonomy/localization/cartographer/mapping/internal/3d/local_slam_result_3d.hpp"

namespace cartographer {
namespace cloud {

template <>
void MapBuilderContext<mapping::Submap2D>::EnqueueLocalSlamResultData(
    int trajectory_id, const std::string& sensor_id,
    const mapping::proto::LocalSlamResultData& local_slam_result_data) {
  map_builder_server_->incoming_data_queue_.Push(
      absl::make_unique<Data>(Data{trajectory_id, absl::make_unique<mapping::LocalSlamResult2D>(
                                                      sensor_id, local_slam_result_data, &submap_controller_)}));
}

template <>
void MapBuilderContext<mapping::Submap3D>::EnqueueLocalSlamResultData(
    int trajectory_id, const std::string& sensor_id,
    const mapping::proto::LocalSlamResultData& local_slam_result_data) {
  map_builder_server_->incoming_data_queue_.Push(
      absl::make_unique<Data>(Data{trajectory_id, absl::make_unique<mapping::LocalSlamResult3D>(
                                                      sensor_id, local_slam_result_data, &submap_controller_)}));
}

}  // namespace cloud
}  // namespace cartographer
