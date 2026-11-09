/* Copyright 2026 The Navigation Authors (navigation) */
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

#include "navigation/runner.hpp"

namespace {

class MockEncoder final : public navigation::perception::ObservationEncoder
{
public:
    navigation::common::Status Encode(
        const navigation::observation::Observation& observation,
        navigation::observation::DepthFeature* feature) override {
        (void)observation;
        feature->add_pooled(1.0F);
        feature->add_pooled(0.0F);
        feature->set_stamp_ns(1);
        feature->set_sequence(1);
        return navigation::common::Status{};
    }

    void Reset() override {}
};

class MockMemory final : public navigation::memory::ContextProvider
{
public:
    navigation::common::Status Insert(
        const navigation::memory::Keyframe&) override {
        ++inserts_;
        return navigation::common::Status{};
    }

    navigation::memory::ContextView View(
        const navigation::common::Pose2D&) const override {
        return {};
    }

    std::size_t Size() const override {
        return inserts_;
    }

    void Reset(bool) override {
        inserts_ = 0;
    }

    navigation::ConsistencyReport CheckConsistency(
        const navigation::common::Pose2D&, std::uint32_t) const override {
        return {};
    }

private:
    std::size_t inserts_{0};
};

class MockPolicy final : public navigation::planning::PolicyInterface
{
public:
    explicit MockPolicy(std::chrono::milliseconds delay) : delay_(delay) {}

    bool Configure(const navigation::config::PolicyOptions&) override {
        return true;
    }

    navigation::common::StatusOr<navigation::NavigationIntent> Query(
        const navigation::observation::Observation&,
        const navigation::observation::DepthFeature&,
        const navigation::GoalSpec&,
        const navigation::memory::ContextView&) override {
        std::this_thread::sleep_for(delay_);
        navigation::NavigationIntent intent;
        auto* chunk = intent.mutable_chunk();
        chunk->set_dt_s(0.1);
        for (int i = 0; i < 8; ++i) {
            auto* action = chunk->add_actions();
            action->set_x(0.4);
            action->set_theta(0.0);
        }
        return intent;
    }

    void Reset() override {}

    std::string_view Name() const override {
        return "mock";
    }

private:
    std::chrono::milliseconds delay_;
};

navigation::config::NavigationConfig MakeConfig() {
    navigation::config::NavigationConfig configuration;
    configuration.mutable_policy()->set_name("vntpa");
    configuration.mutable_policy()->mutable_vntpa()->set_decision_rate_hz(5.0);
    configuration.mutable_rule()->add_enabled_rules("speed_limit");
    configuration.mutable_rule()->add_enabled_rules("safety_chain");
    return configuration;
}

navigation::GoalSpec MakeGoal() {
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

struct QueryGate {
    std::mutex mutex;
    std::condition_variable cv;
    int entered{0};
    int completed{0};
    int released{0};

    bool WaitFor(int count, bool completion = false) {
        std::unique_lock<std::mutex> lock(mutex);
        return cv.wait_for(lock, std::chrono::seconds(2), [&] {
            return (completion ? completed : entered) >= count;
        });
    }

    void Release(int count) {
        const std::lock_guard<std::mutex> lock(mutex);
        released = count;
        cv.notify_all();
    }
};

class GatedPolicy final : public navigation::planning::PolicyInterface
{
public:
    explicit GatedPolicy(std::shared_ptr<QueryGate> gate)
        : gate_(std::move(gate)) {}
    bool Configure(const navigation::config::PolicyOptions&) override {
        return true;
    }
    void Reset() override { cached_goal_.clear(); }
    std::string_view Name() const override {
        return "gated";
    }

    navigation::common::StatusOr<navigation::NavigationIntent> Query(
        const navigation::observation::Observation&,
        const navigation::observation::DepthFeature&,
        const navigation::GoalSpec& goal,
        const navigation::memory::ContextView&) override {
        std::unique_lock<std::mutex> lock(gate_->mutex);
        const int number = ++gate_->entered;
        gate_->cv.notify_all();
        if (!gate_->cv.wait_for(lock, std::chrono::seconds(2),
                                [&] { return gate_->released >= number; })) {
            return navigation::common::Status::Unavailable(
                "test query gate timed out");
        }
        // Model a policy that caches a selected frontier for its goal session.
        if (cached_goal_.empty()) cached_goal_ = goal.goal_id();
        navigation::NavigationIntent intent;
        intent.mutable_chunk()->set_dt_s(0.1);
        for (int i = 0; i < 8; ++i)
            intent.mutable_chunk()->add_actions()->set_x(
                cached_goal_ == "A" ? 0.4 : -0.3);
        ++gate_->completed;
        gate_->cv.notify_all();
        return intent;
    }

private:
    std::shared_ptr<QueryGate> gate_;
    std::string cached_goal_;
};

bool CheckGoalInvalidation(bool cancel_first, bool result_completed) {
    using namespace navigation;  // NOLINT
    auto gate = std::make_shared<QueryGate>();
    Runner::Dependencies dependencies;
    dependencies.encoder = std::make_unique<MockEncoder>();
    dependencies.memory = std::make_unique<MockMemory>();
    dependencies.policy = std::make_unique<GatedPolicy>(gate);
    dependencies.blender = std::make_unique<planning::ChunkBlender>(
        planning::ChunkBlender::Options{8, 2, 0.1, 2.0, 0.0, 0.0});
    auto config = MakeConfig();
    config.mutable_policy()->mutable_vntpa()->set_decision_rate_hz(1.0);
    Runner runner(config, std::move(dependencies));
    // Ensure every failure path releases the query before Runner joins it.
    struct ReleaseOnExit {
        std::shared_ptr<QueryGate> gate;
        ~ReleaseOnExit() {
            gate->Release(std::numeric_limits<int>::max());
        }
    } release{gate};
    if (!runner.Start())
        return false;
    auto goal = MakeGoal();
    goal.set_goal_id("A");
    runner.SetGoal(goal);
    const observation::Observation observation;
    const common::Pose2D pose;
    const common::Twist2D velocity;
    runner.Tick(observation, pose, velocity);
    if (!gate->WaitFor(1))
        return false;
    if (result_completed) {
        gate->Release(1);
        if (!gate->WaitFor(1, true))
            return false;
    }
    if (cancel_first) {
        runner.ClearGoal();
        if (runner.Tick(observation, pose, velocity).has_value())
            return false;
    }
    goal.set_goal_id("B");
    // Action-server threads are allowed to change the goal.
    std::thread setter([&] { runner.SetGoal(goal); });
    setter.join();
    gate->Release(1);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (true) {
        const auto command = runner.Tick(observation, pose, velocity);
        if (!command.has_value() || std::abs(command->x()) > 1e-6) {
            std::cerr << "old goal result reached the new goal\n";
            return false;
        }
        {
            const std::lock_guard<std::mutex> lock(gate->mutex);
            if (gate->entered >= 2)
                break;
        }
        if (std::chrono::steady_clock::now() > deadline)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    gate->Release(2);
    while (std::chrono::steady_clock::now() <= deadline) {
        const auto command = runner.Tick(observation, pose, velocity);
        if (command.has_value() && command->x() < -0.29)
            return true;
        if (command.has_value() && command->x() > 1e-6)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

bool CheckRecoveryOwnershipAndBudget() {
    using namespace navigation;
    std::int64_t now = 1000000000LL;
    auto gate = std::make_shared<QueryGate>();
    Runner::Dependencies dependencies;
    dependencies.encoder = std::make_unique<MockEncoder>();
    dependencies.memory = std::make_unique<MockMemory>();
    dependencies.policy = std::make_unique<GatedPolicy>(gate);
    dependencies.blender = std::make_unique<planning::ChunkBlender>(
        planning::ChunkBlender::Options{8, 2, 0.1, 2.0, 0.0, 0.0});
    dependencies.steady_now_ns = [&] { return now; };
    auto config = MakeConfig();
    config.mutable_behavior()->set_enabled(true);
    config.mutable_behavior()->set_max_recovery_attempts(2);
    Runner runner(config, std::move(dependencies));
    struct ReleaseOnExit {
        std::shared_ptr<QueryGate> gate;
        ~ReleaseOnExit() {
            gate->Release(std::numeric_limits<int>::max());
        }
    } release{gate};
    if (!runner.Start())
        return false;
    auto goal = MakeGoal();
    goal.set_goal_id("A");
    runner.SetGoal(goal);
    const observation::Observation observation;  // Simulated stamp/epoch zero.
    const common::Pose2D pose;
    const common::Twist2D velocity;
    runner.Tick(observation, pose, velocity);
    if (!gate->WaitFor(1))
        return false;
    now += 8100000000LL;
    auto command = runner.Tick(observation, pose, velocity);
    if (runner.RecoveryStatus() != RECOVERY_REQUIRED || !command ||
        std::abs(command->x()) > 1e-6)
        return false;
    if (!runner.RequestRecovery() || runner.RequestRecovery() ||
        runner.RecoveryCount() != 1)
        return false;
    gate->Release(1);
    if (!gate->WaitFor(1, true))
        return false;
    now += 100000000LL;
    command = runner.Tick(observation, pose, velocity, false);
    if (!command || std::abs(command->theta()) > 1e-6)
        return false;
    command = runner.Tick(observation, pose, velocity);
    if (!command || command->theta() < 0.5 || std::abs(command->x()) > 1e-6)
        return false;
    // The completed old policy result must not override the recovery turn.
    now += 8100000000LL;
    command = runner.Tick(observation, pose, velocity);
    if (runner.RecoveryStatus() != RECOVERY_FAILED || !command ||
        std::abs(command->theta()) > 1e-6)
        return false;
    if (!runner.RequestRecovery() || runner.RecoveryCount() != 2)
        return false;
    now += 100000000LL;
    command = runner.Tick(observation, pose, velocity);
    if (!command || command->theta() > -0.5)
        return false;
    now += 8100000000LL;
    runner.Tick(observation, pose, velocity);
    if (runner.RecoveryStatus() != RECOVERY_FAILED || runner.RequestRecovery())
        return false;
    runner.FailGoal();
    command = runner.Tick(observation, pose, velocity);
    if (!runner.IsGoalFailed() || !command ||
        std::abs(command->x()) + std::abs(command->theta()) > 1e-6)
        return false;
    runner.ClearGoal();
    if (runner.HasActiveGoal() || runner.IsGoalFailed() ||
        runner.RecoveryCount() != 0 || runner.Tick(observation, pose, velocity))
        return false;
    goal.set_goal_id("B");
    runner.SetGoal(goal);
    if (runner.RecoveryStatus() != RECOVERY_IDLE || runner.RecoveryCount() != 0)
        return false;
    // Cancellation during a running maneuver clears its state immediately.
    now += 100000000LL;
    runner.Tick(observation, pose, velocity);
    now += 8100000000LL;
    runner.Tick(observation, pose, velocity);
    if (!runner.RequestRecovery())
        return false;
    runner.ClearGoal();
    if (runner.RecoveryStatus() != RECOVERY_IDLE ||
        runner.Tick(observation, pose, velocity))
        return false;
    return true;
}

}  // namespace

int main() {
    using namespace navigation;  // NOLINT

    Runner::Dependencies dependencies;
    dependencies.encoder = std::make_unique<MockEncoder>();
    dependencies.memory = std::make_unique<MockMemory>();
    dependencies.policy =
        std::make_unique<MockPolicy>(std::chrono::milliseconds(30));
    dependencies.blender = std::make_unique<planning::ChunkBlender>(
        planning::ChunkBlender::Options{8, 2, 0.1, 2.0, 0.0, 0.0});
    auto scenario = std::make_unique<planning::ScenarioManager>();
    if (!scenario->Configure(MakeConfig())) {
        std::cerr << "scenario configure failed\n";
        return 1;
    }
    dependencies.scenario = std::move(scenario);

    Runner runner(MakeConfig(), std::move(dependencies));
    if (!runner.Start()) {
        std::cerr << "start failed\n";
        return 1;
    }
    runner.SetGoal(MakeGoal());

    observation::Observation observation;
    common::Pose2D pose;
    const common::Twist2D velocity;
    std::optional<common::Twist2D> command;

    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < 8; ++i) {
        command = runner.Tick(observation, pose, velocity);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    if (elapsed > std::chrono::milliseconds(400)) {
        std::cerr << "tick blocked on policy\n";
        return 1;
    }
    if (!command.has_value()) {
        std::cerr << "no command produced\n";
        return 1;
    }
    if (std::abs(command->x() - 0.32) > 0.02) {
        std::cerr << "scenario rule chain not applied: " << command->x()
                  << "\n";
        return 1;
    }

    pose.set_x(10.0);
    runner.Tick(observation, pose, velocity);
    if (!runner.IsGoalReached()) {
        std::cerr << "goal not reached\n";
        return 1;
    }
    const auto reached_command = runner.Tick(observation, pose, velocity);
    if (!reached_command.has_value() || std::abs(reached_command->x()) > 1e-6) {
        std::cerr << "reached goal continued to move\n";
        return 1;
    }

    if (runner.MemoryEntryCount() == 0) {
        std::cerr << "memory not updated\n";
        return 1;
    }

    runner.Stop();
    if (runner.Tick(observation, pose, velocity).has_value()) {
        std::cerr << "tick after stop returned a command\n";
        return 1;
    }
    for (const bool cancel : {false, true})
        for (const bool completed : {false, true})
            if (!CheckGoalInvalidation(cancel, completed))
                return 1;
    if (!CheckRecoveryOwnershipAndBudget()) {
        std::cerr
            << "recovery ownership, stale policy, lease or budget failed\n";
        return 1;
    }
    return 0;
}
