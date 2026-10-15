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
 * @file controller.hpp
 * @brief PolicyController plugin: learning-based local Twist controller.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/std_msgs/float32_multi_array.pb.h>

#include "autonomy/control/controller/policy_controller/types.hpp"

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/policy_controller/pipeline.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class PolicyController
 * @brief Thin @ref common::ControllerInterface adapter over @ref PolicyPipeline.
 *
 * Responsibilities:
 * - Wire ControllerServer lifecycle / plan / speed limit into the pipeline
 * - Cache goal pose for @c IsGoalReached (path is pushed only on @c SetPlan)
 * - Forward optional sensor observations
 */
class PolicyController : public common::ControllerInterface
{
public:
    PolicyController() = default;
    ~PolicyController() override = default;

    /**
     * @brief Configure plugin options and optionally preload the policy.
     * @param options Controller options (reads @c policy_controller_options).
     * @param name Plugin instance name.
     * @param tf_buffer Shared TF buffer (unused by current pipeline).
     * @param costmap_wrapper Local costmap for crop / collision check.
     */
    void Configure(const proto::ControllerOptions& options, std::string name,
                   std::shared_ptr<transform::Buffer> tf_buffer,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper);

    /**
     * @brief Release plugin resources.
     */
    void Cleanup();
    /**
     * @brief Activate the plugin after Configure.
     */
    void Activate();
    /**
     * @brief Deactivate the plugin before Cleanup.
     */
    void Deactivate();

    /**
     * @brief One control tick: encode → infer → decode → safety filter.
     * @return Status code from @ref CommandSafetyFilter.
     */
    uint32 ComputeVelocityCommands(
        const automsgs::msgs::geometry_msgs::PoseStamped& pose,
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
        automsgs::msgs::geometry_msgs::TwistStamped& command_velocity,
        common::GoalChecker* goal_checker, std::string& message) override;

    /**
     * @brief Goal check using cached goal pose and latest ego pose.
     * @param distance_tolerance Override; <=0 uses options default.
     * @param angle_tolerance Override; <=0 uses options default.
     */
    bool IsGoalReached(double distance_tolerance,
                       double angle_tolerance) override;

    /**
     * @brief Store goal pose and publish path into the observation buffer.
     */
    void SetPlan(const automsgs::msgs::nav_msgs::Path& plan) override;

    /**
     * @brief Update speed scale for the safety filter.
     * @param speed_limit Absolute limit or percentage depending on @p percentage.
     * @param percentage If true, @p speed_limit is percent of max (0–100).
     */
    void SetSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

    /**
     * @brief Clear goal cache and reset the pipeline.
     */
    void Reset() override;

    /**
     * @brief Inject latest laser scan (optional modality).
     */
    void SetLaserScan(const automsgs::msgs::sensor_msgs::LaserScan& laser_scan);
    /**
     * @brief Inject latest camera image (optional modality).
     */
    void SetImage(const automsgs::msgs::sensor_msgs::Image& image);
    /**
     * @brief Inject latest PointCloud2 (optional; light xyz pack).
     */
    void SetPointCloud(
        const automsgs::msgs::sensor_msgs::PointCloud2& point_cloud);
    /**
     * @brief Inject precomputed lidar BEV float grid.
     */
    void SetLidarBev(const TensorFrame& lidar_bev);
    /**
     * @brief Inject a named RAW float tensor (name = ObservationInput.name).
     */
    void SetRawTensor(const TensorFrame& raw);
    /**
     * @brief Pack Float32MultiArray into lidar BEV using options shapes/name.
     */
    void SetLidarBevArray(
        const automsgs::msgs::std_msgs::Float32MultiArray& array);
    /**
     * @brief Pack Float32MultiArray into a RAW tensor using options shapes/name.
     */
    void SetRawTensorArray(
        const automsgs::msgs::std_msgs::Float32MultiArray& array);

    /**
     * @brief Hot-load a policy config from a textproto / binary path.
     * @return false on parse / engine / validation failure.
     */
    bool LoadConfig(const std::string& config_path,
                    std::string* error = nullptr);
    /**
     * @brief Hot-load from an in-memory proto.
     */
    bool LoadConfig(const proto::PolicyConfig& config,
                    std::string* error = nullptr);

    /**
     * @brief Background load; swaps active model on success.
     * @return false if another async load is already in flight.
     */
    bool LoadConfigAsync(const std::string& config_path,
                         std::function<void(bool, const std::string&)> done);
    bool LoadConfigAsync(const proto::PolicyConfig& config,
                         std::function<void(bool, const std::string&)> done);

    /**
     * @brief Access the underlying pipeline (tests / advanced wiring).
     */
    PolicyPipeline& pipeline() { return pipeline_; }
    const PolicyDiagnostics& diagnostics() const {
        return pipeline_.diagnostics();
    }

private:
    std::string plugin_name_;
    PolicyPipeline pipeline_;
    std::optional<automsgs::msgs::geometry_msgs::Pose> goal_pose_;
    std::optional<automsgs::msgs::geometry_msgs::Twist> last_velocity_;
    std::mutex goal_mutex_;
    double control_period_seconds_{0.05};
    double speed_limit_scale_{1.0};
    double near_goal_scale_{1.0};
    std::string model_load_error_;
    common::GoalChecker* last_goal_checker_{nullptr};
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
