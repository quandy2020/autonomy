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

/**
 * @file types.hpp
 * @brief Shared observation POD types for policy_controller.
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/diagnostic_msgs/diagnostic_status.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

#include "autonomy/control/proto/policy_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @struct CostmapCrop
 * @brief Dense local costmap patch already normalized to [0, 1].
 *
 * Produced by @ref ObservationBuffer::UpdateCostmapFrom; avoids OccupancyGrid
 * wire format on the encode path.
 */
struct CostmapCrop {
    /**
     * @brief Cells in x.
     */
    int width{0};
    /**
     * @brief Cells in y.
     */
    int height{0};
    /**
     * @brief Meters per cell.
     */
    float resolution{0.0f};
    /**
     * @brief Row-major, size = width * height.
     */
    std::vector<float> data;
};

/**
 * @struct TensorFrame
 * @brief Precomputed float tensor for RAW / lidar BEV injection.
 *
 * Heavy preprocess (voxel, full BEVFusion) belongs in the exported graph;
 * C++ only forwards these frames.
 */
struct TensorFrame {
    /**
     * @brief Engine input name (must match ObservationInput.name for RAW).
     */
    std::string name;
    /**
     * @brief Tensor shape (product must equal data.size when non-empty).
     */
    std::vector<int64_t> shape;
    /**
     * @brief Row-major float32 payload.
     */
    std::vector<float> data;
    /**
     * @brief Frame stamp in seconds (0 → skip skew check).
     */
    double stamp_sec{0.0};
};

/**
 * @struct ObservationSnapshot
 * @brief One-tick observation view used by encoders / safety filter.
 *
 * Heavy modalities are @c shared_ptr frames so @ref ObservationBuffer::Snapshot
 * is an O(1) pointer copy. Pose / velocity stay by value (small).
 */
struct ObservationSnapshot {
    /**
     * @brief Latest ego pose (by value).
     */
    std::optional<automsgs::msgs::geometry_msgs::PoseStamped> pose;
    /**
     * @brief Latest ego velocity (by value).
     */
    std::optional<automsgs::msgs::geometry_msgs::TwistStamped> velocity;
    /**
     * @brief Shared global path frame.
     */
    std::shared_ptr<const automsgs::msgs::nav_msgs::Path> path;
    /**
     * @brief Shared local costmap crop ([0, 1]).
     */
    std::shared_ptr<const CostmapCrop> costmap;
    /**
     * @brief Shared laser-scan frame.
     */
    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> laser_scan;
    /**
     * @brief Shared camera image frame.
     */
    std::shared_ptr<const automsgs::msgs::sensor_msgs::Image> image;
    /**
     * @brief Shared PointCloud2 frame.
     */
    std::shared_ptr<const automsgs::msgs::sensor_msgs::PointCloud2> point_cloud;
    /**
     * @brief Precomputed lidar BEV grid (single slot).
     */
    std::shared_ptr<const TensorFrame> lidar_bev;
    /**
     * @brief Named RAW tensors (key = ObservationInput.name).
     */
    std::unordered_map<std::string, std::shared_ptr<const TensorFrame>>
        raw_tensors;
    /**
     * @brief Primary stamp (pose, else laser / point cloud).
     */
    double stamp_sec{0.0};
    /**
     * @brief Wall-clock seconds of last good sample per ObservationInput.name.
     */
    std::unordered_map<std::string, double> modality_last_wall_sec;
};

/**
 * @struct ObservationEncodeContext
 * @brief Runtime encoder knobs (not part of the policy sidecar proto).
 */
struct ObservationEncodeContext {
    /**
     * @brief Sampled path points in robot frame.
     */
    int path_point_count{20};
    /**
     * @brief Downsampled range bins.
     */
    int laser_scan_bin_count{180};
    /**
     * @brief Max xyz points packed for point-cloud modality.
     */
    int point_cloud_points{2048};
    /**
     * @brief <=0 disables age checks vs primary stamp.
     */
    double max_observation_age_sec{0.0};
    /**
     * @brief <=0 disables |modality_stamp - pose_stamp| skew checks.
     */
    double max_observation_skew_sec{0.0};
    /**
     * @brief Brief-drop grace window (seconds); <=0 disables.
     */
    double missing_modality_grace_sec{0.0};
    /**
     * @brief Policy when required modality is missing inside grace.
     */
    proto::MissingModalityPolicy missing_modality_policy{
        proto::MISSING_MODALITY_FAIL};
    /**
     * @brief Wall-clock seconds used for grace (steady / system).
     */
    double now_sec{0.0};
    /**
     * @brief Encode kinematic relative to path goal when true.
     */
    bool relative_kinematic{false};
    /**
     * @brief Stride-subsample point cloud instead of first-N.
     */
    bool point_cloud_stride_sample{false};
    /**
     * @brief Runtime cap on diffusion steps (<=0 → sidecar).
     */
    int max_diffusion_steps{0};
};

/**
 * @struct InferBudget
 * @brief Wall-clock deadline for one encode+infer tick (optional).
 */
struct InferBudget {
    double timeout_sec{0.0};
    int max_diffusion_steps{0};
    std::chrono::steady_clock::time_point start{
        std::chrono::steady_clock::now()};
    /** Filled by @ref PredictTwist when timing is measured. */
    double encode_elapsed_ms{0.0};
    double infer_elapsed_ms{0.0};

    bool Enabled() const { return timeout_sec > 0.0; }

    bool Exceeded() const {
        if (!Enabled()) {
            return false;
        }
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          start)
                .count();
        return elapsed > timeout_sec;
    }

    double RemainingSec() const {
        if (!Enabled()) {
            return 0.0;
        }
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          start)
                .count();
        return std::max(0.0, timeout_sec - elapsed);
    }

    double ElapsedMs() const {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - start)
            .count();
    }
};

/**
 * @struct PolicyDiagnostics
 * @brief Per-tick observability counters for offline / log diagnostics.
 */
struct PolicyDiagnostics {
    double last_infer_ms{0.0};
    double last_encode_ms{0.0};
    uint32_t last_status{0};
    std::string last_error;
    std::string last_missing_modality;
    bool skew_or_age_fail{false};
    bool clamp_or_rate_limit{false};
    bool collision_blocked{false};
    bool near_goal_scaled{false};
    bool zero_filled_modality{false};
    bool infer_timeout{false};
    bool frame_align_fail{false};
    uint32_t consecutive_failures{0};
    uint64_t tick_count{0};
    uint64_t success_count{0};
    uint64_t failure_count{0};
};

/**
 * @brief Fill a DiagnosticStatus from @p diagnostics (for topic export).
 */
inline void FillDiagnosticStatus(
    const PolicyDiagnostics& diagnostics, const std::string& name,
    automsgs::msgs::diagnostic_msgs::DiagnosticStatus* status) {
    if (status == nullptr) {
        return;
    }
    status->Clear();
    status->set_name(name.empty() ? "policy_controller" : name);
    status->set_hardware_id("policy_controller");
    status->set_message(diagnostics.last_error.empty() ? "ok"
                                                       : diagnostics.last_error);
    if (diagnostics.infer_timeout || diagnostics.collision_blocked ||
        diagnostics.frame_align_fail) {
        status->set_level(
            automsgs::msgs::diagnostic_msgs::DiagnosticStatus::ERROR);
    } else if (diagnostics.skew_or_age_fail ||
               diagnostics.consecutive_failures > 0 ||
               diagnostics.zero_filled_modality) {
        status->set_level(
            automsgs::msgs::diagnostic_msgs::DiagnosticStatus::WARN);
    } else {
        status->set_level(
            automsgs::msgs::diagnostic_msgs::DiagnosticStatus::OK);
    }
    auto add = [status](const char* key, const std::string& value) {
        auto* kv = status->add_values();
        kv->set_key(key);
        kv->set_value(value);
    };
    add("infer_ms", std::to_string(diagnostics.last_infer_ms));
    add("encode_ms", std::to_string(diagnostics.last_encode_ms));
    add("status", std::to_string(diagnostics.last_status));
    add("consecutive_failures",
        std::to_string(diagnostics.consecutive_failures));
    add("tick_count", std::to_string(diagnostics.tick_count));
    add("success_count", std::to_string(diagnostics.success_count));
    add("failure_count", std::to_string(diagnostics.failure_count));
    add("near_goal_scaled", diagnostics.near_goal_scaled ? "1" : "0");
    add("clamp_or_rate_limit",
        diagnostics.clamp_or_rate_limit ? "1" : "0");
    add("collision_blocked", diagnostics.collision_blocked ? "1" : "0");
    add("infer_timeout", diagnostics.infer_timeout ? "1" : "0");
    add("missing_modality", diagnostics.last_missing_modality);
}

/**
 * @brief Convert @c std_msgs/Header stamp to seconds.
 * @param header Message header with sec / nanosec.
 * @return Absolute time in seconds.
 */
inline double HeaderStampSeconds(
    const automsgs::msgs::std_msgs::Header& header) {
    return static_cast<double>(header.stamp().sec()) +
           static_cast<double>(header.stamp().nanosec()) * 1e-9;
}

/**
 * @brief Whether @p snapshot contains data for @p modality.
 * @param raw_name Required when modality is RAW (ObservationInput.name).
 * @return true if the modality can be encoded this tick.
 */
inline bool HasObservationModality(const ObservationSnapshot& snapshot,
                                   proto::ObservationModality modality,
                                   const std::string& raw_name = {}) {
    switch (modality) {
        case proto::OBSERVATION_KINEMATIC:
            return snapshot.pose.has_value() && snapshot.velocity.has_value();
        case proto::OBSERVATION_PATH:
            return snapshot.path != nullptr && !snapshot.path->poses().empty();
        case proto::OBSERVATION_COSTMAP:
            return snapshot.costmap != nullptr && !snapshot.costmap->data.empty();
        case proto::OBSERVATION_LASER:
            return snapshot.laser_scan != nullptr &&
                   !snapshot.laser_scan->ranges().empty();
        case proto::OBSERVATION_IMAGE:
            return snapshot.image != nullptr && !snapshot.image->data().empty();
        case proto::OBSERVATION_POINT_CLOUD:
            return snapshot.point_cloud != nullptr &&
                   !snapshot.point_cloud->data().empty();
        case proto::OBSERVATION_LIDAR_BEV:
            return snapshot.lidar_bev != nullptr &&
                   !snapshot.lidar_bev->data.empty();
        case proto::OBSERVATION_RAW: {
            if (raw_name.empty()) {
                return !snapshot.raw_tensors.empty();
            }
            const auto it = snapshot.raw_tensors.find(raw_name);
            return it != snapshot.raw_tensors.end() && it->second != nullptr &&
                   !it->second->data.empty();
        }
        default:
            return false;
    }
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
