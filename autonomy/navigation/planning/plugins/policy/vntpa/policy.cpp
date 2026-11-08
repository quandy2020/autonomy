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

#include "navigation/planning/plugins/policy/vntpa/policy.hpp"

#include <string>
#include <utility>

#include "navigation/common/pose2d_convert.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

Policy::Policy(plugins::VntpaPolicyOptions options,
               std::unique_ptr<ModelLoader> model_loader,
               std::unique_ptr<Preprocessor> preprocessor,
               std::unique_ptr<Postprocessor> postprocessor,
               std::unique_ptr<ActionDecoder> action_decoder,
               std::unique_ptr<ManipulationReachabilityAdjuster> adjuster)
    : options_(std::move(options)),
      model_loader_(std::move(model_loader)),
      preprocessor_(std::move(preprocessor)),
      postprocessor_(std::move(postprocessor)),
      action_decoder_(std::move(action_decoder)),
      adjuster_(std::move(adjuster))
{
}

bool Policy::Configure(const config::PolicyOptions& options)
{
    if (options.name() != "vntpa" || !options.has_vntpa())
    {
        return false;
    }
    options_ = options.vntpa();
    if (model_loader_ == nullptr || preprocessor_ == nullptr || postprocessor_ == nullptr ||
        action_decoder_ == nullptr)
    {
        return false;
    }
    std::string error;
    if (!model_loader_->Load(&error))
    {
        configured_ = false;
        return false;
    }
    configured_ = true;
    return true;
}

common::StatusOr<NavigationIntent> Policy::Query(
    const observation::Observation& observation,
    const observation::DepthFeature& depth_feature, const GoalSpec& goal,
    const memory::ContextView& context)
{
    if (!configured_)
    {
        return common::Status::Unavailable("VNTPA policy is not configured");
    }
    common::Pose2D current_pose;
    if (observation.has_odom() && observation.odom().has_pose())
    {
        current_pose = common::FromProto(observation.odom().pose().pose());
    }

    ModelInputs inputs;
    if (!preprocessor_->Build(depth_feature, current_pose, goal, context, &inputs))
    {
        return common::Status::InvalidArgument("VNTPA preprocessor failed");
    }
    VntpaModel* model = model_loader_->Model();
    if (model == nullptr)
    {
        return common::Status::Unavailable("VNTPA model is not loaded");
    }
    std::vector<float> raw_actions;
    std::uint32_t chunk_length = 0;
    if (!model->Run(inputs, &raw_actions, &chunk_length))
    {
        return common::Status::Internal("VNTPA inference failed");
    }

    std::vector<float> decoded;
    if (!action_decoder_->Decode(raw_actions, chunk_length, &decoded))
    {
        return common::Status::Internal("VNTPA action decoder failed");
    }

    NavigationIntent intent;
    if (!postprocessor_->Build(decoded, chunk_length, depth_feature.sequence(),
                               depth_feature.stamp_ns(), intent.mutable_chunk()))
    {
        return common::Status::Internal("VNTPA postprocessor failed");
    }
    intent.set_sequence(depth_feature.sequence());
    intent.set_stamp_ns(depth_feature.stamp_ns());
    intent.set_frame("base_link");
    intent.set_confidence(1.0F - depth_feature.uncertainty());

    if (options_.enable_reachability_adjuster() && adjuster_ != nullptr && goal.has_pose())
    {
        common::Pose2D goal_pose;
        goal_pose.set_x(goal.pose().pose().position().x());
        goal_pose.set_y(goal.pose().pose().position().y());
        for (int i = 0; i < intent.chunk().actions_size(); ++i)
        {
            common::Twist2D command = navigation::common::MakeTwist2D(intent.chunk().actions(i).x(),
                                    intent.chunk().actions(i).y(),
                                    intent.chunk().actions(i).theta());
            if (adjuster_->Adjust(current_pose, goal_pose, &command))
            {
                auto* action = intent.mutable_chunk()->mutable_actions(i);
                action->set_x(command.x());
                action->set_y(command.y());
                action->set_theta(command.theta());
            }
        }
    }
    return intent;
}

void Policy::Reset()
{
    if (model_loader_ != nullptr)
    {
        // Backend state is reset by the next Load in the real implementation.
    }
}

std::string_view Policy::Name() const
{
    return "vntpa";
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
