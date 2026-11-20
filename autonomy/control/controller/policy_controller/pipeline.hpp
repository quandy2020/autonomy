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
#include "autonomy/control/controller/policy_controller/observation/defm_features.hpp"
#include "autonomy/control/controller/policy_controller/observation/frame_align.hpp"
#include "autonomy/control/controller/policy_controller/runtime.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/options.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/native_spline_tracker.hpp"
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

    /** @brief Configure a native reference tracker with the server's existing options and map. */
    void ConfigureTrajectoryTracker(const proto::ControllerOptions& options, const std::string& name,
                                    std::shared_ptr<transform::Buffer> tf_buffer,
                                    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
        native_tracker_.Configure(options, name, std::move(tf_buffer), std::move(costmap));
    }
    /** @brief Activate the owned existing NMPC/MPPI implementation. */
    void ActivateTrajectoryTracker() { native_tracker_.Activate(); }
    /** @brief Cancel the reference before deactivating the native controller. */
    void DeactivateTrajectoryTracker() { native_tracker_.Deactivate(); }
    /** @brief Release native controller resources at plugin cleanup. */
    void CleanupTrajectoryTracker() { native_tracker_.Cleanup(); }
    /** @brief Read the selected reference as the existing automsgs Path type. */
    const automsgs::msgs::nav_msgs::Path& selected_sand_path() const { return native_tracker_.reference_path(); }

    bool LoadConfig(const std::string& config_path,
                    std::string* error = nullptr) {
        if (!policy_manager_.LoadFromFile(config_path, error)) {
            return false;
        }
        RefreshSafetyLimits();
        RefreshSandPlanner();
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
        RefreshSandPlanner();
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
                    RefreshSandPlanner();
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
        native_tracker_.Reset();
        runtime_buffers_.reuse_sand_reference = false;
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
    /**
     * @brief Inject shared frozen depth features into an explicitly trained RAW input.
     * @param features Original shared encoder result; never independently re-encoded here.
     * @param input_name Active policy input receiving these tokens.
     * @param tensor_name Official patch_tokens or global_feature output.
     * @param now_sec Current acquisition-compatible time, seconds.
     * @param error Explicit rejection of missing model, incompatible shape or stale frame.
     * @return True only after strict conversion into the existing observation buffer.
     */
    bool SetDepthFeatures(const perception::defm::DefmDepthFeatures& features,
                          const std::string& input_name, const std::string& tensor_name,
                          double now_sec, std::string* error = nullptr) {
        const auto model = policy_manager_.Active();
        if (!model || options_.missing_modality_policy() == proto::MISSING_MODALITY_ZERO_FILL) {
            if (error) *error = "DeFM requires an active adapted model without zero-filled observations";
            return false;
        }
        for (const auto& input : model->config().proto().inputs()) {
            if (input.name() != input_name) continue;
            TensorFrame frame;
            if (!MakeDefmTensorFrame(features, input, tensor_name, now_sec, &frame, error)) return false;
            observation_buffer_.SetRawTensor(frame);
            return true;
        }
        if (error) *error = "DeFM input is not declared by the active model: " + input_name;
        return false;
    }
    /** @brief Supply complete shared DeFM history to the currently declared token model input.
     * @param history Original immutable depth features, oldest first; no independent encoding occurs.
     * @param input_name Exact active-model RAW input receiving [1,T,N,D].
     * @param now_sec Current acquisition-clock time; the earliest original expiry remains authoritative.
     * @param error Optional shape/time/calibration or model replacement rejection.
     * @return True only after complete valid history is installed in the existing buffer.
     * @details Caller serializes pipeline lifecycle, model configuration and
     * computation. A denied update replaces this named RAW slot with an empty
     * input, so a previously accepted history cannot mask a later invalid one.
     * Generic RAW injection and the model loader do not grant motion admission.
     */
    bool SetDepthFeatureHistory(
        const std::vector<std::shared_ptr<const perception::defm::DefmDepthFeatures>>& history,
        const std::string& input_name, double now_sec, std::string* error = nullptr) {
        if (error) error->clear();
        if (input_name.empty()) {
            if (error) *error = "DeFM history input name is empty";
            return false;
        }
        TensorFrame frame;
        const auto reject = [&](const std::string& reason) {
            TensorFrame invalid; invalid.name = input_name;
            observation_buffer_.SetRawTensor(invalid);
            if (error) *error = reason;
            return false;
        };
        const auto model = policy_manager_.Active();
        if (!model || options_.missing_modality_policy() == proto::MISSING_MODALITY_ZERO_FILL)
            return reject("DeFM history requires an active adapted model without zero-filled observations");
        for (const auto& input : model->config().proto().inputs()) {
            if (input.name() != input_name) continue;
            std::string reason;
            if (!MakeDefmHistoryTensorFrame(history, input, now_sec, &frame, &reason)) return reject(reason);
            if (policy_manager_.Active() != model) return reject("DeFM history model changed during input preparation");
            observation_buffer_.SetRawTensor(frame);
            return true;
        }
        return reject("DeFM history input is not declared by the active model: " + input_name);
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
        // A new path resets planning/controller state, not the latest measured
        // pose, velocity, image/depth, costmap or other observation modalities.
        // Clearing them here made the first control tick after every FollowPath
        // start fail with a missing required modality.
        safety_filter_.Reset();
        infer_watchdog_.Reset();
        runtime_buffers_.inputs.clear();
        runtime_buffers_.outputs.clear();
        runtime_buffers_.action_sequence.clear();
        runtime_buffers_.selected_sand_controls.clear();
        runtime_buffers_.reuse_sand_reference = false;
        native_tracker_.Reset();
        sand_model_.reset();
        runtime_buffers_.previous_plan_heading = Eigen::Vector3d::Zero();
        runtime_buffers_.has_previous_plan_heading = false;
        encode_context_.has_previous_plan_heading = false;
        encode_context_.previous_plan_heading = Eigen::Vector3d::Zero();
        diagnostics_ = PolicyDiagnostics{};
        last_tick_ = std::chrono::steady_clock::now();
    }

    void ClearModels() {
        infer_watchdog_.Reset();
        native_tracker_.Reset();
        runtime_buffers_.selected_sand_controls.clear();
        runtime_buffers_.reuse_sand_reference = false;
        sand_model_.reset();
        policy_manager_.Clear();
    }

    /**
     * @brief Full control tick: observation → policy → native SanD tracker → safety.
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
        // Message acquisition stamps and modality receive ages are wall-clock
        // seconds. The monotonic clock above is used only for elapsed budgets.
        encode_context_.now_sec = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

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
        RefreshSandPlanner();

        const bool is_sand = model->config().is_sand_planner();
        if (is_sand && sand_model_.lock() != model) {
            native_tracker_.Reset();
            runtime_buffers_.selected_sand_controls.clear();
            sand_model_ = model;
        }
        if (is_sand && !native_tracker_.ValidateFeedback(pose, velocity, encode_context_.now_sec, &error)) {
            return RecordFailure(error, command_velocity, message, true);
        }
        if (is_sand && command_velocity) command_velocity->mutable_header()->set_frame_id(options_.robot_frame());
        runtime_buffers_.reuse_sand_reference = is_sand && native_tracker_.CanReuse(now);

        ObservationSnapshot raw = observation_buffer_.Snapshot();
        ObservationSnapshot snapshot;
        if (!AlignObservationFrames(raw, tf_buffer_, options_.robot_frame(),
                                    options_.transform_tolerance(),
                                    options_.require_frame_match(), &snapshot,
                                    &error)) {
            diagnostics_.frame_align_fail = true;
            return RecordFailure(error, command_velocity, message, is_sand);
        }

        encode_context_.has_previous_plan_heading =
            runtime_buffers_.has_previous_plan_heading;
        encode_context_.previous_plan_heading =
            runtime_buffers_.previous_plan_heading;

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
            return RecordFailure(error, command_velocity, message, timed_out || is_sand);
        }

        if (!ValidateRawTensorLifetimes(snapshot, model->config(),
                encode_context_.now_sec + std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - now).count(), &error)) {
            return RecordFailure(error, command_velocity, message, true);
        }
        if (is_sand) {
            if (zero_filled) {
                return RecordFailure("SanD Missing required observation; zero-filled input cannot authorize motion",
                                     command_velocity, message, true);
            }
            const auto& sand = model->config().effective_sand_planner();
            if (!runtime_buffers_.reuse_sand_reference &&
                !native_tracker_.SetReference(runtime_buffers_.selected_sand_controls, pose,
                                              sand.waypoint_count(), sand.execute_speed(), &error)) {
                return RecordFailure(error, command_velocity, message, true);
            }
            automsgs::msgs::geometry_msgs::TwistStamped tracked;
            const double tracking_now = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (native_tracker_.Compute(pose, velocity, tracking_now, &tracked, &error) !=
                    proto::CONTROLLER_RESULT_SUCCESS || budget.Exceeded()) {
                if (budget.Exceeded()) error = "SanD native tracking timeout";
                return RecordFailure(error, command_velocity, message, true);
            }
            action.CopyFrom(tracked.twist());
        }
        diagnostics_.last_encode_ms = budget.encode_elapsed_ms;
        diagnostics_.last_infer_ms = budget.infer_elapsed_ms > 0.0
                                         ? budget.infer_elapsed_ms
                                         : budget.ElapsedMs();
        diagnostics_.zero_filled_modality = zero_filled;

        if (!ValidateRawTensorLifetimes(snapshot, model->config(),
                encode_context_.now_sec + std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - now).count(), &error)) {
            return RecordFailure(error, command_velocity, message, true);
        }

        bool clamp = false;
        bool collision = false;
        const uint32_t status = safety_filter_.Apply(
            action, snapshot, costmap_wrapper, elapsed_seconds, command_velocity,
            message, &clamp, &collision);
        if (is_sand && command_velocity) command_velocity->mutable_header()->set_frame_id(options_.robot_frame());
        if (!ValidateRawTensorLifetimes(snapshot, model->config(),
                encode_context_.now_sec + std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - now).count(), &error)) {
            return RecordFailure(error, command_velocity, message, true);
        }
        diagnostics_.clamp_or_rate_limit = clamp;
        diagnostics_.collision_blocked = collision;
        if (is_sand && status != proto::CONTROLLER_RESULT_SUCCESS) {
            native_tracker_.Reset();
            runtime_buffers_.reuse_sand_reference = false;
            runtime_buffers_.has_previous_plan_heading = false;
        }
        diagnostics_.last_status = status;
        diagnostics_.last_error = message != nullptr ? *message : "";
        if (status ==
            static_cast<uint32_t>(proto::CONTROLLER_RESULT_SUCCESS)) {
            ++diagnostics_.success_count;
            diagnostics_.consecutive_failures = 0;
            if (is_sand && command_velocity) {
                runtime_buffers_.previous_plan_heading = Eigen::Vector3d(command_velocity->twist().linear().x(), 0.0, 0.0);
                runtime_buffers_.has_previous_plan_heading = true;
            }
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
    void RefreshSandPlanner() {
        auto model = policy_manager_.Active();
        if (!model) {
            return;
        }
        encode_context_.sand_planner = model->config().effective_sand_planner();
        observation_buffer_.SetDepthHistoryCapacity(
            encode_context_.sand_planner.depth_history());
    }

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
        const bool force_zero = force_zero_hint || trip || !runtime_buffers_.selected_sand_controls.empty();
        native_tracker_.Reset();
        runtime_buffers_.reuse_sand_reference = false;
        runtime_buffers_.has_previous_plan_heading = false;
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
    sand_planner::NativeSplineTracker native_tracker_;  ///< Existing NMPC/MPPI implementation plus automsgs reference adapter.
    std::weak_ptr<const PolicyModel> sand_model_;  ///< Invalidates cached geometry on model replacement.
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
