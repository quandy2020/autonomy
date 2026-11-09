/* Copyright 2026 The Navigation Authors (navigation) */
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <vector>

#include "navigation/behavior/engine/behavior_session.hpp"
#include "navigation/behavior/engine/bt_cpp_engine.hpp"

int main(int argc, char** argv) {
    using namespace navigation;
    const auto root = argc > 1 ? std::filesystem::path(argv[1])
                               : std::filesystem::path(__FILE__)
                                     .parent_path()
                                     .parent_path()
                                     .parent_path();
    setenv("NAVIGATION_CONF_DIR", root.c_str(), 1);
    config::BehaviorOptions options;
    options.set_enabled(true);
    options.set_tree("navigate_with_recovery.xml");
    options.set_max_recovery_attempts(2);
    std::vector<BehaviorRequest> requests;
    behavior::BehaviorSession session([&](const BehaviorRequest& request) {
        requests.push_back(request);
        return true;
    });
    behavior::BtCppEngine engine;
    if (!engine.Configure(options, &session)) {
        std::cerr << engine.Error();
        return 1;
    }
    NavigationStatus status;
    status.set_session_id("navigation-instance-a");
    status.set_goal_epoch(7);
    status.set_has_active_goal(true);
    status.set_pose_valid(true);
    status.mutable_goal()->set_type(GoalSpec::TYPE_POSE);
    status.mutable_goal()->set_goal_id("goal-a");
    auto& context = engine.Context();
    context.goal = status.goal();
    context.request_assist = [&](std::string_view reason) {
        session.FailGoal(reason);
    };
    std::int64_t now = 1000000000LL;
    auto tick = [&] {
        now += 100000000LL;
        session.Update(status, now);
        return engine.Tick(now);
    };
    if (!tick().Ok() || !requests.empty())
        return 2;
    status.set_recovery_state(RECOVERY_REQUIRED);
    for (int i = 0; i < 3 && requests.empty(); ++i)
        if (!tick().Ok())
            return 3;
    if (requests.size() != 1 ||
        requests.back().kind() != BehaviorRequest::RECOVER)
        return 4;
    const auto first = requests.back();
    if (!behavior::MatchesBehaviorRequest(first, status, now))
        return 5;
    // Delay acknowledgement: re-send; all copies become invalid once accepted.
    tick();
    tick();
    if (requests.size() != 2)
        return 6;
    status.set_recovery_count(1);
    status.set_recovery_state(RECOVERY_RUNNING);
    for (int i = 0; i < 20; ++i)
        if (!tick().Ok())
            return 7;
    if (requests.size() != 2 ||
        behavior::MatchesBehaviorRequest(first, status, now))
        return 8;
    status.set_recovery_state(RECOVERY_SUCCEEDED);
    tick();
    tick();
    status.set_goal_reached(true);
    if (!tick().Ok() || context.assist_requested)
        return 9;
    // A new goal resets the XML, and terminal failure is requested once the
    // persistent navigation count has exhausted the budget (also after
    // restart).
    engine.Reset();
    status.set_goal_reached(false);
    status.set_goal_epoch(8);
    status.set_recovery_count(2);
    status.set_recovery_state(RECOVERY_FAILED);
    bool terminal = false;
    for (int i = 0; i < 4 && !terminal; ++i)
        terminal = !tick().Ok();
    if (!terminal || !context.assist_requested ||
        requests.back().kind() != BehaviorRequest::FAIL_GOAL)
        return 10;
    const auto failure = requests.back();
    NavigationStatus newer = status;
    newer.set_goal_epoch(9);
    if (behavior::MatchesBehaviorRequest(failure, newer, now))
        return 11;
    newer = status;
    newer.set_session_id("navigation-instance-b");
    if (behavior::MatchesBehaviorRequest(failure, newer, now))
        return 12;
    newer = status;
    newer.set_has_active_goal(false);
    if (behavior::MatchesBehaviorRequest(failure, newer, now) ||
        behavior::MatchesBehaviorRequest(failure, status, now + 600000000LL))
        return 13;
    options.set_tree("missing-tree.xml");
    if (engine.Configure(options, &session) || engine.Tick(now).Ok())
        return 14;
    std::cout << "PASS real XML registration, asynchronous recovery, "
                 "persistent budget, stale requests and missing XML\n";
    return 0;
}
