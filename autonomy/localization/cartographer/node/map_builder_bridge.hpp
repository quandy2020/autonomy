/*
 * Copyright 2016 The Cartographer Authors
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

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>

#include "autonomy/localization/cartographer/common/port.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder_interface.hpp"
#include "autonomy/localization/cartographer/mapping/pose_graph_interface.hpp"
#include "autonomy/localization/cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "autonomy/localization/cartographer/mapping/trajectory_builder_interface.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/sensor_bridge.hpp"
#include "autonomy/localization/cartographer/node/trajectory_options.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

class MapBuilderBridge {
public:
    struct LocalTrajectoryData {
        struct LocalSlamData {
            ::cartographer::common::Time time;
            ::cartographer::transform::Rigid3d local_pose;
            ::cartographer::sensor::RangeData range_data_in_local;
        };
        std::shared_ptr<const LocalSlamData> local_slam_data;
        ::cartographer::transform::Rigid3d local_to_map;
        std::unique_ptr<::cartographer::transform::Rigid3d> published_to_tracking;
        TrajectoryOptions trajectory_options;
    };

    MapBuilderBridge(
        const NodeOptions& node_options,
        std::unique_ptr<::cartographer::mapping::MapBuilderInterface> map_builder,
        transform::Buffer* tf_buffer);

    MapBuilderBridge(const MapBuilderBridge&) = delete;
    MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

    void LoadState(const std::string& state_filename, bool load_frozen_state);
    int AddTrajectory(
        const std::set<
            ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
            expected_sensor_ids,
        const TrajectoryOptions& trajectory_options);
    void FinishTrajectory(int trajectory_id);
    void RunFinalOptimization();
    bool SerializeState(const std::string& filename,
                        bool include_unfinished_submaps);

    void HandleSubmapQuery(
        const proto::SubmapQueryRequest& request,
        proto::SubmapQueryResponse* response);

    std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
    GetTrajectoryStates();

    proto::SubmapList GetSubmapList(
        const commsgs::builtin_interfaces::Time& node_time);

    std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
        LOCKS_EXCLUDED(mutex_);

    SensorBridge* sensor_bridge(int trajectory_id);

    ::cartographer::mapping::MapBuilderInterface* map_builder() {
        return map_builder_.get();
    }

private:
    void OnLocalSlamResult(
        int trajectory_id, ::cartographer::common::Time time,
        ::cartographer::transform::Rigid3d local_pose,
        ::cartographer::sensor::RangeData range_data_in_local)
        LOCKS_EXCLUDED(mutex_);

    std::mutex mutex_;
    const NodeOptions node_options_;
    std::unordered_map<int,
                       std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
        local_slam_data_ GUARDED_BY(mutex_);
    std::unique_ptr<::cartographer::mapping::MapBuilderInterface> map_builder_;
    transform::Buffer* const tf_buffer_;

    std::unordered_map<int, TrajectoryOptions> trajectory_options_;
    std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
};

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
