/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/frame_align.hpp"

#include <algorithm>
#include <utility>

#include "autonomy/transform/tf2/exceptions.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool FramesEqual(const std::string& a, const std::string& b) {
    return !a.empty() && !b.empty() && a == b;
}

bool ValidateSensorFrame(const std::string& frame_id,
                         const std::string& expected_frame,
                         bool require_frame_match, const char* modality,
                         std::string* error) {
    if (!require_frame_match) {
        return true;
    }
    if (frame_id.empty()) {
        if (error != nullptr) {
            *error = std::string(modality) +
                     " frame_id is empty (require_frame_match=true; "
                     "publishers must set frame_id=" +
                     expected_frame + ")";
        }
        return false;
    }
    if (FramesEqual(frame_id, expected_frame)) {
        return true;
    }
    if (error != nullptr) {
        *error = std::string(modality) + " frame must be '" + expected_frame +
                 "' (got '" + frame_id +
                 "'); C++ does not TF-transform this modality";
    }
    return false;
}

}  // namespace

bool AlignObservationFrames(
    const ObservationSnapshot& snapshot,
    const std::shared_ptr<transform::Buffer>& tf_buffer,
    const std::string& robot_frame, double transform_tolerance,
    bool require_frame_match, ObservationSnapshot* aligned,
    std::string* error) {
    if (aligned == nullptr) {
        return false;
    }
    *aligned = snapshot;

    const std::string expected_sensor =
        robot_frame.empty() ? "base_link" : robot_frame;

    if (aligned->laser_scan != nullptr &&
        !ValidateSensorFrame(aligned->laser_scan->header().frame_id(),
                             expected_sensor, require_frame_match, "LaserScan",
                             error)) {
        return false;
    }
    if (aligned->image != nullptr &&
        !ValidateSensorFrame(aligned->image->header().frame_id(),
                             expected_sensor, require_frame_match, "Image",
                             error)) {
        return false;
    }
    if (aligned->point_cloud != nullptr &&
        !ValidateSensorFrame(aligned->point_cloud->header().frame_id(),
                             expected_sensor, require_frame_match,
                             "PointCloud2", error)) {
        return false;
    }

    if (aligned->path == nullptr || aligned->path->poses().empty() ||
        !aligned->pose.has_value()) {
        return true;
    }

    const std::string pose_frame = aligned->pose->header().frame_id();
    const std::string path_frame = aligned->path->header().frame_id();
    if (path_frame.empty() || pose_frame.empty() ||
        FramesEqual(path_frame, pose_frame)) {
        return true;
    }

    if (!tf_buffer) {
        if (error != nullptr) {
            *error = "Path frame '" + path_frame + "' != pose frame '" +
                     pose_frame + "' and TF buffer is null";
        }
        return false;
    }

    auto transformed = std::make_shared<automsgs::msgs::nav_msgs::Path>();
    *transformed->mutable_header() = aligned->path->header();
    transformed->mutable_header()->set_frame_id(pose_frame);
    const float timeout = static_cast<float>(std::max(0.0, transform_tolerance));
    for (int i = 0; i < aligned->path->poses_size(); ++i) {
        automsgs::msgs::geometry_msgs::PoseStamped source =
            aligned->path->poses(i);
        if (source.header().frame_id().empty()) {
            source.mutable_header()->set_frame_id(path_frame);
        }
        if (FramesEqual(source.header().frame_id(), pose_frame)) {
            *transformed->add_poses() = source;
            continue;
        }
        try {
            automsgs::msgs::geometry_msgs::PoseStamped out;
            // Prefer message stamp when available for better temporal alignment.
            tf_buffer->transform(source, out, pose_frame, timeout);
            *transformed->add_poses() = out;
        } catch (const transform::tf2::TransformException& ex) {
            if (error != nullptr) {
                *error = std::string("Path TF align failed: ") + ex.what();
            }
            return false;
        }
    }
    if (transformed->poses().empty()) {
        if (error != nullptr) {
            *error = "Path TF align produced empty path";
        }
        return false;
    }
    aligned->path = std::move(transformed);
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
