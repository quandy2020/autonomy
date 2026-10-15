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
 * @file pipeline.hpp
 * @brief Encode → infer → decode → safety pipeline.
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/policy_controller/action/safety_filter.hpp"
#include "autonomy/control/controller/policy_controller/manager.hpp"
#include "autonomy/control/controller/policy_controller/observation/buffer.hpp"
#include "autonomy/control/controller/policy_controller/observation/frame_align.hpp"
#include "autonomy/control/controller/policy_controller/runtime.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/control/proto/policy_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class PolicyPipeline
 * @brief Orchestrates one control tick for a learning-based local policy.
 *
 * Hot-path buffers in @ref PolicyRuntimeBuffers are reused across ticks.
 */
class PolicyPipeline
{
public:
    /**
     * @brief Apply plugin options and configure the safety filter.
     */
    void Configure(const proto::PolicyControllerOptions& options) {
        options_ = options;
        ApplyOptionDefaults();
        encode_context_.path_point_count = options_.path_points();
        encode_context_.laser_scan_bin_count = options_.laser_bins();
        encode_context_.point_cloud_points = options_.point_cloud_points();
        encode_context_.max_observation_age_sec = options_.max_obs_age();
        encode_context_.max_observation_skew_sec = options_.max_obs_skew();
        encode_context_.missing_modality_grace_sec =
            options_.missing_modality_grace();
        encode_context_.missing_modality_policy =
            options_.missing_modality_policy();
        encode_context_.relative_kinematic = options_.relative_kinematic();
        encode_context_.point_cloud_stride_sample =
            options_.point_cloud_stride_sample();
        encode_context_.max_diffusion_steps = options_.max_diffusion_steps();
        proto::TwistLimits limits;
        if (options_.has_config() && options_.config().has_action()) {
            limits = options_.config().action().limits();
        }
        safety_filter_.Configure(options_, limits);
        last_tick_ = std::chrono::steady_clock::now();
    }

    void SetTfBuffer(std::shared_ptr<transform::Buffer> tf_buffer) {
        tf_buffer_ = std::move(tf_buffer);
    }

    bool LoadConfig(const std::string& config_path,
                    std::string* error = nullptr) {
        if (!policy_manager_.LoadFromFile(config_path, error)) {
            return false;
        }
        RefreshSafetyLimits();
        return true;
    }

    bool LoadConfig(const proto::PolicyConfig& config,
                    std::string* error = nullptr) {
        PolicyConfig parsed;
        if (!PolicyConfig::LoadFromProto(config, &parsed, error)) {
            return false;
        }
        if (!policy_manager_.Load(parsed, error)) {
            return false;
        }
        RefreshSafetyLimits();
        return true;
    }

    bool LoadConfigAsync(
        PolicyConfig config,
        std::function<void(bool, const std::string&)> done) {
        return policy_manager_.LoadAsync(
            std::move(config),
            [this, done = std::move(done)](bool ok,
                                           const std::string& error) {
                if (ok) {
                    RefreshSafetyLimits();
                }
                if (done) {
                    done(ok, error);
                }
            });
    }

    bool LoadConfigAsync(
        const std::string& config_path,
        std::function<void(bool, const std::string&)> done) {
        PolicyConfig config;
        std::string error;
        if (!PolicyConfig::LoadFromFile(config_path, &config, &error)) {
            if (done) {
                done(false, error);
            }
            return false;
        }
        return LoadConfigAsync(std::move(config), std::move(done));
    }

    /**
     * @brief Load the active model from options if none is loaded yet.
     */
    bool LoadModel(std::string* error = nullptr) {
        if (policy_manager_.HasActive()) {
            return true;
        }
        if (!options_.config_path().empty()) {
            return LoadConfig(options_.config_path(), error);
        }
        if (options_.has_config()) {
            return LoadConfig(options_.config(), error);
        }
        if (error != nullptr) {
            *error = "No config_path or config configured";
        }
        return false;
    }

    void SetPath(const automsgs::msgs::nav_msgs::Path& path) {
        observation_buffer_.SetPath(path);
    }
    void SetLaserScan(
        const automsgs::msgs::sensor_msgs::LaserScan& laser_scan) {
        observation_buffer_.SetLaserScan(laser_scan);
    }
    void SetImage(const automsgs::msgs::sensor_msgs::Image& image) {
        observation_buffer_.SetImage(image);
    }
    void SetPointCloud(
        const automsgs::msgs::sensor_msgs::PointCloud2& point_cloud) {
        observation_buffer_.SetPointCloud(point_cloud);
    }
    void SetLidarBev(const TensorFrame& lidar_bev) {
        observation_buffer_.SetLidarBev(lidar_bev);
    }
    void SetRawTensor(const TensorFrame& raw) {
        observation_buffer_.SetRawTensor(raw);
    }
    void UpdateCostmap(map::costmap_2d::Costmap2DWrapper* costmap_wrapper) {
        observation_buffer_.UpdateCostmapFrom(costmap_wrapper,
                                              options_.costmap_cells());
    }

    void SetSpeedScale(double scale) {
        speed_scale_cache_ = std::max(0.0, scale);
        safety_filter_.SetSpeedScale(speed_scale_cache_);
    }

    void SetNearGoalScaled(bool scaled) {
        diagnostics_.near_goal_scaled = scaled;
    }

    void Reset() {
        observation_buffer_.Clear();
        safety_filter_.Reset();
        infer_watchdog_.Reset();
        runtime_buffers_.inputs.clear();
        runtime_buffers_.outputs.clear();
        runtime_buffers_.action_sequence.clear();
        diagnostics_ = PolicyDiagnostics{};
        last_tick_ = std::chrono::steady_clock::now();
    }

    void ClearModels() {
        infer_watchdog_.Reset();
        policy_manager_.Clear();
    }

    /**
     * @brief Full control tick: ego update → PredictTwist → safety.
     */
    uint32_t ComputeCommand(
        const automsgs::msgs::geometry_msgs::PoseStamped& pose,
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
        map::costmap_2d::Costmap2DWrapper* costmap_wrapper,
        double control_period_seconds,
        automsgs::msgs::geometry_msgs::TwistStamped* command_velocity,
        std::string* message) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - last_tick_).count();
        last_tick_ = now;
        const double elapsed_seconds =
            elapsed > 0.0 && elapsed < 1.0 ? elapsed : control_period_seconds;
        encode_context_.now_sec =
            std::chrono::duration<double>(now.time_since_epoch()).count();

        observation_buffer_.SetEgoState(pose, velocity);
        UpdateCostmap(costmap_wrapper);

        if (command_velocity != nullptr) {
            command_velocity->mutable_header()->CopyFrom(pose.header());
            command_velocity->mutable_twist()->Clear();
        }

        ++diagnostics_.tick_count;
        diagnostics_.skew_or_age_fail = false;
        diagnostics_.clamp_or_rate_limit = false;
        diagnostics_.collision_blocked = false;
        diagnostics_.zero_filled_modality = false;
        diagnostics_.infer_timeout = false;
        diagnostics_.frame_align_fail = false;
        diagnostics_.last_missing_modality.clear();

        std::string error;
        if (!LoadModel(&error)) {
            return RecordFailure(error, command_velocity, message, false);
        }

        auto model = policy_manager_.Active();
        if (!model) {
            return RecordFailure("No active PolicyModel", command_velocity,
                                 message, false);
        }

        ObservationSnapshot raw = observation_buffer_.Snapshot();
        ObservationSnapshot snapshot;
        if (!AlignObservationFrames(raw, tf_buffer_, options_.robot_frame(),
                                    options_.transform_tolerance(),
                                    options_.require_frame_match(), &snapshot,
                                    &error)) {
            diagnostics_.frame_align_fail = true;
            return RecordFailure(error, command_velocity, message, false);
        }

        InferBudget budget;
        budget.timeout_sec = options_.infer_timeout();
        budget.max_diffusion_steps = options_.max_diffusion_steps();
        budget.start = std::chrono::steady_clock::now();

        automsgs::msgs::geometry_msgs::Twist action;
        bool zero_filled = false;
        if (!PredictTwist(model, snapshot, encode_context_, &runtime_buffers_,
                          &action, &error, &zero_filled, &budget,
                          &infer_watchdog_)) {
            diagnostics_.last_encode_ms = budget.encode_elapsed_ms;
            diagnostics_.last_infer_ms = budget.infer_elapsed_ms > 0.0
                                             ? budget.infer_elapsed_ms
                                             : budget.ElapsedMs();
            const bool timed_out =
                error.find("timeout") != std::string::npos || budget.Exceeded();
            diagnostics_.infer_timeout = timed_out;
            if (error.find("Missing") != std::string::npos ||
                error.find("Holding on missing") != std::string::npos) {
                diagnostics_.last_missing_modality = error;
            }
            if (error.find("skew") != std::string::npos ||
                error.find("Stale") != std::string::npos) {
                diagnostics_.skew_or_age_fail = true;
            }
            return RecordFailure(error, command_velocity, message, timed_out);
        }
        diagnostics_.last_encode_ms = budget.encode_elapsed_ms;
        diagnostics_.last_infer_ms = budget.infer_elapsed_ms > 0.0
                                         ? budget.infer_elapsed_ms
                                         : budget.ElapsedMs();
        diagnostics_.zero_filled_modality = zero_filled;

        bool clamp = false;
        bool collision = false;
        const uint32_t status = safety_filter_.Apply(
            action, snapshot, costmap_wrapper, elapsed_seconds, command_velocity,
            message, &clamp, &collision);
        diagnostics_.clamp_or_rate_limit = clamp;
        diagnostics_.collision_blocked = collision;
        diagnostics_.last_status = status;
        diagnostics_.last_error = message != nullptr ? *message : "";
        if (status ==
            static_cast<uint32_t>(proto::CONTROLLER_RESULT_SUCCESS)) {
            ++diagnostics_.success_count;
            diagnostics_.consecutive_failures = 0;
            return status;
        }
        ++diagnostics_.failure_count;
        ++diagnostics_.consecutive_failures;
        const bool trip =
            options_.max_consecutive_failures() > 0 &&
            static_cast<int>(diagnostics_.consecutive_failures) >=
                options_.max_consecutive_failures();
        if (trip) {
            safety_filter_.ApplyFailure(diagnostics_.last_error,
                                        command_velocity, message,
                                        /*force_zero=*/true);
        }
        MaybeLogFailure(diagnostics_.last_error);
        return status;
    }

    ObservationBuffer& observation_buffer() { return observation_buffer_; }
    PolicyManager& policy_manager() { return policy_manager_; }
    const proto::PolicyControllerOptions& options() const { return options_; }
    const PolicyDiagnostics& diagnostics() const { return diagnostics_; }

private:
    void RefreshSafetyLimits() {
        proto::TwistLimits limits;
        auto model = policy_manager_.Active();
        if (model) {
            limits = model->config().proto().action().limits();
        } else if (options_.has_config() && options_.config().has_action()) {
            limits = options_.config().action().limits();
        }
        safety_filter_.Configure(options_, limits);
        safety_filter_.SetSpeedScale(speed_scale_cache_);
    }

    void ApplyOptionDefaults() {
        if (options_.path_points() <= 0) {
            options_.set_path_points(20);
        }
        if (options_.laser_bins() <= 0) {
            options_.set_laser_bins(180);
        }
        if (options_.point_cloud_points() <= 0) {
            options_.set_point_cloud_points(2048);
        }
        if (options_.costmap_cells() < 0) {
            options_.set_costmap_cells(64);
        }
        if (options_.goal_xy_tolerance() <= 0.0) {
            options_.set_goal_xy_tolerance(0.25);
        }
        if (options_.goal_yaw_tolerance() <= 0.0) {
            options_.set_goal_yaw_tolerance(0.25);
        }
        if (options_.collision_distance() <= 0.0) {
            options_.set_collision_distance(0.5);
        }
        if (options_.near_goal_distance() > 0.0 &&
            options_.near_goal_speed_scale() <= 0.0) {
            options_.set_near_goal_speed_scale(0.3);
        }
        if (options_.transform_tolerance() <= 0.0) {
            options_.set_transform_tolerance(0.1);
        }
        if (options_.robot_frame().empty()) {
            options_.set_robot_frame("base_link");
        }
        if (options_.collision_cost_threshold() <= 0.0) {
            options_.set_collision_cost_threshold(253.0);
        }
    }

    uint32_t RecordFailure(
        const std::string& error,
        automsgs::msgs::geometry_msgs::TwistStamped* command_velocity,
        std::string* message, bool force_zero_hint) {
        ++diagnostics_.failure_count;
        ++diagnostics_.consecutive_failures;
        diagnostics_.last_error = error;
        const bool trip =
            options_.max_consecutive_failures() > 0 &&
            static_cast<int>(diagnostics_.consecutive_failures) >=
                options_.max_consecutive_failures();
        const bool force_zero = force_zero_hint || trip;
        const uint32_t status = safety_filter_.ApplyFailure(
            error, command_velocity, message, force_zero);
        diagnostics_.last_status = status;
        MaybeLogFailure(error);
        return status;
    }

    void MaybeLogFailure(const std::string& error) {
        if (error.empty() || error == last_logged_error_) {
            return;
        }
        last_logged_error_ = error;
        AWARN << "PolicyPipeline tick fail: " << error
              << " infer_ms=" << diagnostics_.last_infer_ms
              << " consecutive=" << diagnostics_.consecutive_failures
              << " ticks=" << diagnostics_.tick_count
              << " ok=" << diagnostics_.success_count
              << " fail=" << diagnostics_.failure_count;
    }

    proto::PolicyControllerOptions options_;
    ObservationEncodeContext encode_context_;
    ObservationBuffer observation_buffer_;
    PolicyManager policy_manager_;
    CommandSafetyFilter safety_filter_;
    PolicyRuntimeBuffers runtime_buffers_;
    InferWatchdog infer_watchdog_;
    PolicyDiagnostics diagnostics_;
    std::shared_ptr<transform::Buffer> tf_buffer_;
    double speed_scale_cache_{1.0};
    std::string last_logged_error_;
    std::chrono::steady_clock::time_point last_tick_{};
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
