/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/perception/observation_builder.hpp"

#include <cmath>
#include <string>

#include "automsgs/msgs/time_utils.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/pose2d_convert.hpp"

namespace navigation
{
namespace perception
{
namespace {

common::Pose2D Planar(const automsgs::msgs::geometry_msgs::Transform& transform)
{
    automsgs::msgs::geometry_msgs::Pose pose;
    pose.mutable_position()->set_x(transform.translation().x());
    pose.mutable_position()->set_y(transform.translation().y());
    pose.mutable_position()->set_z(transform.translation().z());
    *pose.mutable_orientation() = transform.rotation();
    return common::FromProto(pose);
}

void ExpressOdomInMap(const automsgs::msgs::geometry_msgs::TransformStamped& map_to_odom,
                      observation::Observation* observation)
{
    auto* odom = observation->mutable_odom();
    const std::string& parent = odom->header().frame_id();
    if (parent.empty() || parent == observation->map_frame()) {
        return;
    }
    if (map_to_odom.header().frame_id() != observation->map_frame() ||
        map_to_odom.child_frame_id() != parent) {
        return;
    }
    const common::Pose2D map_pose =
        common::Compose(Planar(map_to_odom.transform()),
                        common::FromProto(odom->pose().pose()));
    std::int64_t stamp_ns = 0;
    if (odom->pose().pose().header().has_stamp()) {
        stamp_ns = static_cast<std::int64_t>(
            automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
                odom->pose().pose().header().stamp()));
    }
    common::ToProto(map_pose, observation->map_frame(), stamp_ns,
                    odom->mutable_pose()->mutable_pose());
    odom->mutable_header()->set_frame_id(observation->map_frame());
}

}  // namespace

bool ObservationBuilder::Build(const ObservationInputs& inputs,
                               observation::Observation* observation)
{
    if (observation == nullptr || inputs.depth == nullptr ||
        inputs.depth->width() == 0 || inputs.depth->height() == 0 ||
        inputs.depth->data().empty())
    {
        return false;
    }
    observation->Clear();
    *observation->mutable_depth() = *inputs.depth;
    *observation->mutable_header() = inputs.depth->header();
    observation->set_sequence(++sequence_);
    observation->set_map_frame("map");
    observation->set_camera_frame("camera_optical_frame");

    std::uint32_t validity = observation::OBSERVATION_FLAG_DEPTH_VALID;
    if (inputs.camera_info != nullptr)
    {
        *observation->mutable_camera_info() = *inputs.camera_info;
        validity |= observation::OBSERVATION_FLAG_CAMERA_INFO_VALID;
    }
    if (inputs.odom != nullptr)
    {
        *observation->mutable_odom() = *inputs.odom;
        if (inputs.map_to_odom != nullptr) {
            ExpressOdomInMap(*inputs.map_to_odom, observation);
        }
        validity |= observation::OBSERVATION_FLAG_ODOM_VALID;
    }
    if (inputs.imu != nullptr)
    {
        *observation->mutable_imu() = *inputs.imu;
        validity |= observation::OBSERVATION_FLAG_IMU_VALID;
    }
    const bool odom_in_map =
        observation->has_odom() &&
        observation->odom().header().frame_id() == observation->map_frame();
    if (inputs.tf_valid || (inputs.map_to_odom != nullptr && odom_in_map))
    {
        validity |= observation::OBSERVATION_FLAG_TF_VALID;
    }
    if (inputs.map != nullptr) {
        const auto& map = *inputs.map;
        *observation->mutable_map() = map;
        const auto& info = map.info();
        const auto& p = info.origin().position();
        const auto& q = info.origin().orientation();
        const std::uint64_t cells = std::uint64_t(info.width()) * info.height();
        if (cells > 0 && cells <= 1000000 && cells == std::uint64_t(map.data_size()) &&
            std::isfinite(info.resolution()) && info.resolution() > 0 &&
            std::isfinite(p.x()) && std::isfinite(p.y()) &&
            std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) && std::isfinite(q.w()) &&
            map.header().frame_id() == observation->map_frame()) {
            *observation->mutable_map() = map;
            validity |= observation::OBSERVATION_FLAG_MAP_VALID;
        }
    }
    observation->set_validity(validity);
    return true;
}

void ObservationBuilder::Reset()
{
    sequence_ = 0;
}

std::uint64_t ObservationBuilder::LastSequence() const
{
    return sequence_;
}

}  // namespace perception
}  // namespace navigation
