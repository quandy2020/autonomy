/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

/**
 * @file frame_align.hpp
 * @brief TF path alignment and sensor frame validation before encode.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Align path poses into the ego pose frame and validate sensor frames.
 *
 * Frame contract (train / deploy):
 * - Path waypoints and ego pose must share a frame after alignment (pose frame).
 * - Path encoder then converts to robot-body (x,y) relative to ego yaw.
 * - Laser / image / point cloud: if @p require_frame_match, @c frame_id must
 *   be non-empty and equal @p robot_frame (no TF transform for these
 *   modalities in C++; publishers must already use robot_frame).
 * - Kinematic tensor uses absolute pose in the pose frame (document map
 *   origin stability for training).
 *
 * @param[in] snapshot Raw observation snapshot.
 * @param[in] tf_buffer Shared TF buffer (may be null → only exact string match).
 * @param[in] robot_frame Expected sensor frame (empty → "base_link").
 * @param[in] transform_tolerance TF lookup timeout (seconds).
 * @param[in] require_frame_match Enforce sensor frame checks.
 * @param[out] aligned Snapshot with path rewritten into pose frame when needed.
 * @param[out] error Failure reason.
 * @return false on TF / frame mismatch.
 */
bool AlignObservationFrames(
    const ObservationSnapshot& snapshot,
    const std::shared_ptr<transform::Buffer>& tf_buffer,
    const std::string& robot_frame, double transform_tolerance,
    bool require_frame_match, ObservationSnapshot* aligned,
    std::string* error = nullptr);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
