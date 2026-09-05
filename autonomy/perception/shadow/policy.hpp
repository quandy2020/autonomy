/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file policy.hpp
 * @brief Fixed-profile ground motion-primitive inference for Shadow.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_POLICY_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_POLICY_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/shadow/proto/shadow.pb.h"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <memory>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {

class YopoPolicy;

/**
 * @brief Backend-independent fixed-profile inference contract for YopoPolicy.
 */
class PolicyRunner
{
public:
    virtual ~PolicyRunner() = default;

    /**
     * @brief Executes one policy inference request.
     * @param inputs Tensors named `depth`, `robot_state`, and `target_state`.
     * @param outputs Tensors named `trajectories` and `scores` on success.
     * @param error Optional diagnostic output.
     * @return True when inference succeeds.
     */
    virtual bool Run(const common::network::TensorMap& inputs,
                     common::network::TensorMap* outputs,
                     std::string* error = nullptr) = 0;

protected:
    /**
     * @brief Captures fixed model metadata for private startup validation.
     */
    PolicyRunner(std::vector<common::network::ModelTensorInfo> input_infos,
                 std::vector<common::network::ModelTensorInfo> output_infos);

private:
    friend class YopoPolicy;

    std::vector<common::network::ModelTensorInfo> input_infos_;
    std::vector<common::network::ModelTensorInfo> output_infos_;
};

/**
 * @brief Builds ground-robot policy inputs and decodes base-frame candidates.
 *
 * The returned paths preserve model candidate and step order. Their parallel
 * scores retain the model convention that smaller values are preferred.
 */
class YopoPolicy
{
public:
    /**
     * @brief Creates the common-network-backed policy for a fixed profile.
     * @param options Validated Shadow runtime configuration.
     * @param error Optional diagnostic output, cleared on entry.
     * @return A ready policy, or nullptr when setup or metadata validation
     * fails.
     */
    static std::unique_ptr<YopoPolicy> Create(
        const proto::ShadowOptions& options, std::string* error = nullptr);

    /**
     * @brief Creates a policy with an injected runner.
     * @param options Validated Shadow runtime configuration.
     * @param runner Inference runner whose metadata must match the profile.
     * @param error Optional diagnostic output, cleared on entry.
     * @return A ready policy, or nullptr when setup or metadata validation
     * fails.
     */
    static std::unique_ptr<YopoPolicy> Create(
        const proto::ShadowOptions& options,
        std::unique_ptr<PolicyRunner> runner, std::string* error = nullptr);

    YopoPolicy(const YopoPolicy&) = delete;
    YopoPolicy& operator=(const YopoPolicy&) = delete;
    YopoPolicy(YopoPolicy&&) = delete;
    YopoPolicy& operator=(YopoPolicy&&) = delete;

    /**
     * @brief Infers candidate `(x, y, yaw)` paths in configured base frame.
     *
     * `depth` is metric `32FC1`. Odometry supplies map-frame robot pose and
     * base-frame linear-x/angular-z velocity. Target pose and planar velocity
     * are supplied in the configured map frame and rotated into the robot
     * frame before inference. Outputs are cleared on every failure.
     *
     * @param depth Metric depth image.
     * @param odometry Robot map pose and base-frame differential-drive state.
     * @param target Map-frame selected-target pose.
     * @param target_velocity Map-frame selected-target planar velocity.
     * @param paths Candidate paths in the configured base frame.
     * @param scores Candidate scores parallel to @p paths; lower is better.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when inputs, inference, and all outputs are valid.
     */
    bool Generate(
        const automsgs::msgs::sensor_msgs::Image& depth,
        const automsgs::msgs::nav_msgs::Odometry& odometry,
        const automsgs::msgs::geometry_msgs::PoseStamped& target,
        const automsgs::msgs::geometry_msgs::TwistStamped& target_velocity,
        std::vector<automsgs::msgs::nav_msgs::Path>* paths,
        std::vector<float>* scores, std::string* error = nullptr);

private:
    YopoPolicy(proto::ShadowOptions options,
               std::unique_ptr<PolicyRunner> runner);

    proto::ShadowOptions options_;
    std::unique_ptr<PolicyRunner> runner_;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_POLICY_HPP_
