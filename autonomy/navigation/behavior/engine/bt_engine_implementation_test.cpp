/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/behavior/engine/bt_engine_implementation.hpp"

namespace
{

class MockSession final : public navigation::planning::NavigationSession
{
public:
    bool SetGoal(const navigation::GoalSpec&) override
    {
        active = true;
        return true;
    }

    void ClearGoal() override
    {
        active = false;
    }

    bool HasActiveGoal() const override
    {
        return active;
    }

    bool IsGoalReached() const override
    {
        return reached;
    }

    bool RequestRecovery(std::string_view) override
    {
        ++recoveries;
        return accept_recovery;
    }

    navigation::lifecycle::ModuleState State() const override
    {
        return module_state;
    }

    bool active{false};
    bool reached{false};
    bool accept_recovery{true};
    int recoveries{0};
    navigation::lifecycle::ModuleState module_state{
        navigation::lifecycle::MODULE_STATE_READY};
};

}  // namespace

int main()
{
    navigation::config::BehaviorOptions options;
    options.set_enabled(true);
    options.set_max_recovery_attempts(2);

    MockSession session;
    navigation::behavior::BtEngineImplementation engine;
    if (!engine.Configure(options, &session))
    {
        std::cerr << "configure failed\n";
        return 1;
    }
    if (!engine.Tick(1).Ok())
    {
        std::cerr << "idle tick failed\n";
        return 1;
    }

    session.active = true;
    if (!engine.Tick(2).Ok())
    {
        std::cerr << "navigate tick failed\n";
        return 1;
    }
    session.reached = true;
    engine.Tick(3);
    if (!session.reached)
    {
        std::cerr << "goal state lost\n";
        return 1;
    }

    session.reached = false;
    session.module_state = navigation::lifecycle::MODULE_STATE_FAILED;
    engine.Tick(4);
    if (session.recoveries != 1)
    {
        std::cerr << "recovery not requested\n";
        return 1;
    }

    session.module_state = navigation::lifecycle::MODULE_STATE_FAILED;
    engine.Tick(5);
    if (session.recoveries != 2)
    {
        std::cerr << "second recovery not requested\n";
        return 1;
    }
    session.module_state = navigation::lifecycle::MODULE_STATE_FAILED;
    if (engine.Tick(6).Ok())
    {
        std::cerr << "recovery exhaustion not reported\n";
        return 1;
    }
    return 0;
}
