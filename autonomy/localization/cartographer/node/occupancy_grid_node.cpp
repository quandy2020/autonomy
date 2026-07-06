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

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/localization/cartographer/node/occupancy_grid_node.hpp"

#include <chrono>

#include "glog/logging.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {
namespace {

using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

int64_t MsFromSeconds(const double seconds) {
    return static_cast<int64_t>(seconds * 1000.0);
}

}  // namespace

OccupancyGridNode::OccupancyGridNode(const double resolution,
                                     const double publish_period_sec,
                                     const bool include_frozen_submaps,
                                     const bool include_unfrozen_submaps)
    : resolution_(resolution),
      publish_period_sec_(publish_period_sec),
      include_frozen_submaps_(include_frozen_submaps),
      include_unfrozen_submaps_(include_unfrozen_submaps) {}

bool OccupancyGridNode::Init(std::shared_ptr<autolink::Node> node) {
    node_ = std::move(node);
    submap_query_client_ =
        node_->CreateClient<proto::SubmapQueryRequest, proto::SubmapQueryResponse>(
            kSubmapQueryServiceName);
    occupancy_grid_writer_ =
        node_->CreateWriter<commsgs::map_msgs::OccupancyGrid>(kOccupancyGridTopic);

    node_->CreateReader<proto::SubmapList>(
        kSubmapListTopic,
        [this](const std::shared_ptr<proto::SubmapList>& msg) {
            HandleSubmapList(msg);
        });

    publish_timer_ = std::make_unique<autolink::Timer>(
        MsFromSeconds(publish_period_sec_), [this]() { DrawAndPublish(); }, false);
    publish_timer_->Start();
    return true;
}

void OccupancyGridNode::HandleSubmapList(
    const std::shared_ptr<proto::SubmapList>& msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<SubmapId> submap_ids_to_delete;
    for (const auto& pair : submap_slices_) {
        submap_ids_to_delete.insert(pair.first);
    }

    const auto timeout = std::chrono::milliseconds(
        static_cast<int>(publish_period_sec_ * 1000.0));

    for (const auto& submap_msg : msg->submap()) {
        const SubmapId id{submap_msg.trajectory_id(), submap_msg.submap_index()};
        submap_ids_to_delete.erase(id);
        if ((submap_msg.is_frozen() && !include_frozen_submaps_) ||
            (!submap_msg.is_frozen() && !include_unfrozen_submaps_)) {
            continue;
        }
        SubmapSlice& submap_slice = submap_slices_[id];
        submap_slice.pose = ToRigid3d(FromProtoPose(submap_msg.pose()));
        submap_slice.metadata_version = submap_msg.submap_version();
        if (submap_slice.surface != nullptr &&
            submap_slice.version == submap_msg.submap_version()) {
            continue;
        }

        auto fetched_textures =
            FetchSubmapTextures(id, submap_query_client_, timeout);
        if (!fetched_textures || fetched_textures->textures.empty()) {
            continue;
        }
        submap_slice.version = fetched_textures->version;
        const auto& fetched_texture = fetched_textures->textures.front();
        submap_slice.width = fetched_texture.width;
        submap_slice.height = fetched_texture.height;
        submap_slice.slice_pose = fetched_texture.slice_pose;
        submap_slice.resolution = fetched_texture.resolution;
        submap_slice.cairo_data.clear();
        submap_slice.surface = ::cartographer::io::DrawTexture(
            fetched_texture.pixels.intensity, fetched_texture.pixels.alpha,
            fetched_texture.width, fetched_texture.height, &submap_slice.cairo_data);
    }

    for (const auto& id : submap_ids_to_delete) {
        submap_slices_.erase(id);
    }

    if (msg->has_header()) {
        last_timestamp_ = commsgs::builtin_interfaces::FromProto(msg->header().stamp());
        last_frame_id_ = msg->header().frame_id();
    }
}

void OccupancyGridNode::DrawAndPublish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (submap_slices_.empty() || last_frame_id_.empty()) {
        return;
    }
    auto painted_slices =
        ::cartographer::io::PaintSubmapSlices(submap_slices_, resolution_);
    auto grid = CreateOccupancyGridMsg(painted_slices, resolution_, last_frame_id_,
                                       last_timestamp_);
    if (grid) {
        occupancy_grid_writer_->Write(*grid);
    }
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
