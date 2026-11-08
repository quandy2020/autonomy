/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "navigation/planning/plugins/policy/vntpa/policy.hpp"

namespace
{

class FakeModel final : public navigation::planning::vntpa::VntpaModel
{
public:
    bool Run(const navigation::planning::vntpa::ModelInputs& inputs,
             std::vector<float>* actions, std::uint32_t* chunk_length) override
    {
        if (inputs.kf_tokens.empty() || inputs.context_size == 0 ||
            inputs.query_pose.size() != 3 || inputs.goal.size() != 2)
        {
            return false;
        }
        *chunk_length = 8;
        actions->assign(24, 0.0F);
        for (std::uint32_t i = 0; i < 8; ++i)
        {
            (*actions)[3 * i] = 0.2F;
            (*actions)[3 * i + 2] = 0.1F;
        }
        return true;
    }
};

class FakeLoader final : public navigation::planning::vntpa::ModelLoader
{
public:
    explicit FakeLoader(const navigation::plugins::VntpaPolicyOptions& options)
        : ModelLoader(options)
    {
    }

    bool Load(std::string*) override
    {
        model_ = std::make_unique<FakeModel>();
        return true;
    }
};

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    plugins::VntpaPolicyOptions options;
    options.set_action_chunk_length(8);
    options.set_execute_length(2);
    options.set_decision_rate_hz(5.0);
    options.set_action_dt_s(0.1);
    options.set_context_size_k(4);

    auto policy = std::make_unique<planning::vntpa::Policy>(
        options, std::make_unique<FakeLoader>(options),
        std::make_unique<planning::vntpa::Preprocessor>(4, 8, 8),
        std::make_unique<planning::vntpa::Postprocessor>(
            planning::vntpa::PostprocessorOptions{8, 0.1, 0.5, 0.0, 1.0}),
        std::make_unique<planning::vntpa::ActionDecoder>());

    config::PolicyOptions policy_options;
    policy_options.set_name("vntpa");
    *policy_options.mutable_vntpa() = options;
    if (!policy->Configure(policy_options))
    {
        std::cerr << "configure failed\n";
        return 1;
    }

    observation::Observation observation;
    observation.mutable_odom()->mutable_pose()->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
    observation::DepthFeature feature;
    feature.add_tokens(0.1F);
    feature.add_tokens(0.2F);
    feature.add_pooled(1.0F);
    feature.set_token_count(1);
    feature.set_token_dim(2);
    feature.set_sequence(3);
    feature.set_stamp_ns(100);
    feature.set_uncertainty(0.1F);

    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(1.0);
    goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);

    const auto result = policy->Query(observation, feature, goal, memory::ContextView{});
    if (!result.Ok() || !result.Value().has_chunk() ||
        result.Value().chunk().actions_size() != 8)
    {
        std::cerr << "query failed\n";
        return 1;
    }
    return 0;
}
