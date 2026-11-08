/* Copyright 2026 The Navigation Authors (navigation) */
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "navigation/runner.hpp"

namespace
{

class MockEncoder final : public navigation::perception::ObservationEncoder
{
public:
    navigation::common::Status Encode(
        const navigation::observation::Observation& observation,
        navigation::observation::DepthFeature* feature) override
    {
        (void)observation;
        feature->add_pooled(1.0F);
        feature->add_pooled(0.0F);
        feature->set_stamp_ns(1);
        feature->set_sequence(1);
        return navigation::common::Status{};
    }

    void Reset() override
    {
    }
};

class MockMemory final : public navigation::memory::ContextProvider
{
public:
    navigation::common::Status Insert(const navigation::memory::Keyframe&) override
    {
        ++inserts_;
        return navigation::common::Status{};
    }

    navigation::memory::ContextView View(const navigation::common::Pose2D&) const override
    {
        return {};
    }

    std::size_t Size() const override
    {
        return inserts_;
    }

    void Reset(bool) override
    {
        inserts_ = 0;
    }

    navigation::ConsistencyReport CheckConsistency(
        const navigation::common::Pose2D&, std::uint32_t) const override
    {
        return {};
    }

private:
    std::size_t inserts_{0};
};

class MockPolicy final : public navigation::planning::PolicyInterface
{
public:
    explicit MockPolicy(std::chrono::milliseconds delay) : delay_(delay)
    {
    }

    bool Configure(const navigation::config::PolicyOptions&) override
    {
        return true;
    }

    navigation::common::StatusOr<navigation::NavigationIntent> Query(
        const navigation::observation::Observation&,
        const navigation::observation::DepthFeature&, const navigation::GoalSpec&,
        const navigation::memory::ContextView&) override
    {
        std::this_thread::sleep_for(delay_);
        navigation::NavigationIntent intent;
        auto* chunk = intent.mutable_chunk();
        chunk->set_dt_s(0.1);
        for (int i = 0; i < 8; ++i)
        {
            auto* action = chunk->add_actions();
            action->set_x(0.4);
            action->set_theta(0.0);
        }
        return intent;
    }

    void Reset() override
    {
    }

    std::string_view Name() const override
    {
        return "mock";
    }

private:
    std::chrono::milliseconds delay_;
};

navigation::config::NavigationConfig MakeConfig()
{
    navigation::config::NavigationConfig configuration;
    configuration.mutable_policy()->set_name("vntpa");
    configuration.mutable_policy()->mutable_vntpa()->set_decision_rate_hz(5.0);
    configuration.mutable_rule()->add_enabled_rules("speed_limit");
    configuration.mutable_rule()->add_enabled_rules("safety_chain");
    return configuration;
}

navigation::GoalSpec MakeGoal()
{
    navigation::GoalSpec goal;
    goal.set_type(navigation::GoalSpec::TYPE_POSE);
    goal.set_position_tolerance(0.15);
    goal.set_yaw_tolerance_rad(0.15);
    auto* pose = goal.mutable_pose();
    pose->mutable_header()->set_frame_id("map");
    pose->mutable_pose()->mutable_position()->set_x(10.0);
    pose->mutable_pose()->mutable_orientation()->set_w(1.0);
    return goal;
}

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    Runner::Dependencies dependencies;
    dependencies.encoder = std::make_unique<MockEncoder>();
    dependencies.memory = std::make_unique<MockMemory>();
    dependencies.policy = std::make_unique<MockPolicy>(std::chrono::milliseconds(30));
    dependencies.blender = std::make_unique<planning::ChunkBlender>(
        planning::ChunkBlender::Options{8, 2, 0.1, 2.0, 0.0, 0.0});
    auto scenario = std::make_unique<planning::ScenarioManager>();
    if (!scenario->Configure(MakeConfig()))
    {
        std::cerr << "scenario configure failed\n";
        return 1;
    }
    dependencies.scenario = std::move(scenario);

    Runner runner(MakeConfig(), std::move(dependencies));
    if (!runner.Start())
    {
        std::cerr << "start failed\n";
        return 1;
    }
    runner.SetGoal(MakeGoal());

    observation::Observation observation;
    common::Pose2D pose;
    const common::Twist2D velocity;
    std::optional<common::Twist2D> command;

    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < 8; ++i)
    {
        command = runner.Tick(observation, pose, velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    if (elapsed > std::chrono::milliseconds(400))
    {
        std::cerr << "tick blocked on policy\n";
        return 1;
    }
    if (!command.has_value())
    {
        std::cerr << "no command produced\n";
        return 1;
    }
    if (std::abs(command->x() - 0.32) > 0.02)
    {
        std::cerr << "scenario rule chain not applied: " << command->x() << "\n";
        return 1;
    }

    pose.set_x(10.0);
    runner.Tick(observation, pose, velocity);
    if (!runner.IsGoalReached())
    {
        std::cerr << "goal not reached\n";
        return 1;
    }

    if (runner.MemoryEntryCount() == 0)
    {
        std::cerr << "memory not updated\n";
        return 1;
    }

    runner.Stop();
    if (runner.Tick(observation, pose, velocity).has_value())
    {
        std::cerr << "tick after stop returned a command\n";
        return 1;
    }
    return 0;
}
