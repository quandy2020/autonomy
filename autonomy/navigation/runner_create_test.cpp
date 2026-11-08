/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <memory>

#include "navigation/planning/plugin_manager.hpp"
#include "navigation/runner.hpp"

namespace
{

class FakeEncoder final : public navigation::perception::ObservationEncoder
{
public:
    navigation::common::Status Encode(
        const navigation::observation::Observation&,
        navigation::observation::DepthFeature* feature) override
    {
        feature->add_pooled(1.0F);
        feature->set_token_count(0);
        feature->set_token_dim(1);
        feature->set_stamp_ns(1);
        return navigation::common::Status{};
    }

    void Reset() override
    {
    }
};

class FakePolicy final : public navigation::planning::PolicyInterface
{
public:
    bool Configure(const navigation::config::PolicyOptions&) override
    {
        return true;
    }

    navigation::common::StatusOr<navigation::NavigationIntent> Query(
        const navigation::observation::Observation&,
        const navigation::observation::DepthFeature&, const navigation::GoalSpec&,
        const navigation::memory::ContextView&) override
    {
        navigation::NavigationIntent intent;
        auto* chunk = intent.mutable_chunk();
        chunk->set_dt_s(0.1);
        for (int i = 0; i < 8; ++i)
        {
            chunk->add_actions()->set_x(0.1);
        }
        return intent;
    }

    void Reset() override
    {
    }

    std::string_view Name() const override
    {
        return "fake";
    }
};

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    auto& plugins = planning::PluginManager::Instance();
    plugins.Clear();
    plugins.RegisterEncoder("defm", [](const config::NavigationConfig&) {
        return std::make_unique<FakeEncoder>();
    });
    plugins.RegisterPolicy("vntpa", [](const config::NavigationConfig&) {
        return std::make_unique<FakePolicy>();
    });

    config::NavigationConfig configuration;
    configuration.mutable_policy()->set_name("vntpa");
    configuration.mutable_policy()->mutable_vntpa()->set_decision_rate_hz(5.0);
    configuration.mutable_policy()->mutable_vntpa()->set_action_chunk_length(8);
    configuration.mutable_policy()->mutable_vntpa()->set_execute_length(2);
    configuration.mutable_policy()->mutable_vntpa()->set_action_dt_s(0.1);
    configuration.mutable_memory()->set_name("pose_indexed");
    configuration.mutable_memory()->mutable_pose_indexed()->set_max_entries(10);
    configuration.mutable_memory()->mutable_pose_indexed()->set_min_insert_interval_s(0.0);
    configuration.mutable_rule()->add_enabled_rules("speed_limit");

    auto runner = Runner::Create(configuration);
    if (runner == nullptr)
    {
        std::cerr << "create failed\n";
        return 1;
    }
    if (!runner->Start())
    {
        std::cerr << "start failed\n";
        return 1;
    }
    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(10.0);
    goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
    runner->SetGoal(goal);

    observation::Observation observation;
    common::Pose2D pose;
    const common::Twist2D velocity;
    for (int i = 0; i < 10; ++i)
    {
        (void)runner->Tick(observation, pose, velocity);
    }
    runner->Stop();
    plugins.Clear();
    return 0;
}
