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

#include "autolink/autolink.hpp"
#include "autolink/timer/timer.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/localization/cartographer/common/port.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/mapping/id.hpp"
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/node_constants.hpp"
#include "autonomy/localization/cartographer/node/submap_fetch.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

class OccupancyGridNode {
public:
    OccupancyGridNode(double resolution, double publish_period_sec,
                      bool include_frozen_submaps,
                      bool include_unfrozen_submaps);

    bool Init(std::shared_ptr<autolink::Node> node);

private:
    void HandleSubmapList(const std::shared_ptr<proto::SubmapList>& msg);
    void DrawAndPublish();

    const double resolution_;
    const double publish_period_sec_;
    const bool include_frozen_submaps_;
    const bool include_unfrozen_submaps_;

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Client<proto::SubmapQueryRequest,
                                     proto::SubmapQueryResponse>>
        submap_query_client_;
    std::shared_ptr<autolink::Writer<commsgs::map_msgs::OccupancyGrid>>
        occupancy_grid_writer_;

    std::mutex mutex_;
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices_ GUARDED_BY(mutex_);
    std::string last_frame_id_ GUARDED_BY(mutex_);
    commsgs::builtin_interfaces::Time last_timestamp_ GUARDED_BY(mutex_);
    std::unique_ptr<autolink::Timer> publish_timer_;
};

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
