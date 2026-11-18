/* Copyright 2026 The Openbot Authors. */
/** @file native_route_protocol.hpp
 * @brief Lossless graph evidence conversion using the existing automsgs GetPlan service.
 */
#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <automsgs/srvs/get_plan.pb.h>
#include "autonomy/planning/planner/vista/route_planner.hpp"

namespace autonomy::planning::planner::vista {

/** @brief Canonical route endpoint service; distinct from the navigation action. */
inline constexpr char kGetPlanServiceName[] = "/get_plan";

/** @brief Convert one planar endpoint into the existing PoseStamped wire type. */
inline automsgs::msgs::geometry_msgs::PoseStamped RoutePose(const Pose2d& pose,
                                                          const std::string& frame) {
    automsgs::msgs::geometry_msgs::PoseStamped result;
    result.mutable_header()->set_frame_id(frame);
    result.mutable_pose()->mutable_position()->set_x(pose.x);
    result.mutable_pose()->mutable_position()->set_y(pose.y);
    result.mutable_pose()->mutable_orientation()->set_z(std::sin(pose.yaw / 2));
    result.mutable_pose()->mutable_orientation()->set_w(std::cos(pose.yaw / 2));
    return result;
}

/** @brief Encode both source sequences independently; no scalar maximum replaces them. */
inline void EncodePrefix(const perception::ObservationCut& cut,
                         automsgs::srvs::GetPlan::ObservationPrefix* result) {
    result->set_present(cut.present);
    result->set_lidar_sequence(cut.sequence[0]);
    result->set_rgbd_sequence(cut.sequence[1]);
}

/** @brief Reject unsupported source bits and hidden sequences in absent sources. */
inline std::optional<perception::ObservationCut> DecodePrefix(
    const automsgs::srvs::GetPlan::ObservationPrefix& value) {
    if (value.present() > 3 || (!(value.present() & 1) && value.lidar_sequence()) ||
        (!(value.present() & 2) && value.rgbd_sequence())) return {};
    perception::ObservationCut result;
    result.present = static_cast<perception::SensorMask>(value.present());
    result.sequence = {value.lidar_sequence(), value.rgbd_sequence()};
    return result;
}

/** @brief Copy owner-supplied provenance without synthesizing fusion acknowledgements. */
inline void EncodeEvidence(const VistaMapEvidence& evidence,
                          automsgs::srvs::GetPlan::GraphEvidence* result) {
    result->set_frame_id(evidence.frame_id);
    result->set_map_epoch(evidence.map_epoch);
    result->set_geometry_revision(evidence.geometry_revision);
    result->set_observation_watermark(evidence.watermark);
    EncodePrefix(evidence.observation_cut, result->mutable_observation_cut());
    EncodePrefix(evidence.observation_baseline, result->mutable_observation_baseline());
    result->set_observation_recovery_generation(evidence.observation_recovery_generation);
}

/** @brief Require complete frame/version metadata and a consistent recovery prefix. */
inline std::optional<VistaMapEvidence> DecodeEvidence(
    const automsgs::srvs::GetPlan::GraphEvidence& value) {
    const auto cut = DecodePrefix(value.observation_cut());
    const auto baseline = DecodePrefix(value.observation_baseline());
    if (value.frame_id().empty() || !value.map_epoch() || !value.geometry_revision() ||
        !value.has_observation_cut() || !value.has_observation_baseline() ||
        !cut || !baseline || !cut->Covers(*baseline) ||
        (baseline->present && !value.observation_recovery_generation())) return {};
    return VistaMapEvidence{value.map_epoch(), value.geometry_revision(),
        value.observation_watermark(), *cut, *baseline,
        value.observation_recovery_generation(), value.frame_id()};
}

}  // namespace autonomy::planning::planner::vista
