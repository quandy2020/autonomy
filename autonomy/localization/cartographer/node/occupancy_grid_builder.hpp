/*
 * Copyright 2026 The Openbot Authors
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
#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/map_msgs/map_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/occupancy_grid.pb.h>
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/mapping/id.hpp"
#include "autonomy/localization/cartographer/mapping/proto/trajectory.pb.h"
#include "autonomy/localization/cartographer/mapping/trajectory_node.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

bool UpdateSubmapSliceFromTextures(
    ::cartographer::io::SubmapSlice* slice,
    const ::cartographer::io::SubmapTextures& textures,
    const automsgs::msgs::geometry_msgs::Pose& submap_pose);

bool UpdateSubmapSliceFromQueryResponse(
    ::cartographer::io::SubmapSlice* slice,
    const proto::SubmapQueryResponse& response,
    const automsgs::msgs::geometry_msgs::Pose& submap_pose,
    int submap_list_version);

std::unique_ptr<automsgs::msgs::map_msgs::OccupancyGrid> BuildOccupancyGrid(
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>& slices,
    double resolution, const std::string& frame_id,
    const automsgs::msgs::builtin_interfaces::Time& stamp);

std::vector<::cartographer::mapping::proto::Trajectory> BuildTrajectoryProtos(
    const ::cartographer::mapping::MapById<::cartographer::mapping::NodeId,
                                           ::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes);

/** Save painted submaps as map.pgm, map.yaml and map.png under output_directory. */
bool SaveMapImageFiles(
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>& slices,
    double resolution, const std::string& output_directory,
    const std::string& filestem,
    const std::vector<::cartographer::mapping::proto::Trajectory>&
        trajectories = {});

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
