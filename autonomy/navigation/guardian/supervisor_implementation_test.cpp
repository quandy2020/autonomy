/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <vector>

#include "navigation/guardian/supervisor_implementation.hpp"

namespace
{

navigation::lifecycle::Heartbeat MakeHeartbeat(const char* module, std::uint64_t sequence,
                                               std::int64_t stamp_ns)
{
    navigation::lifecycle::Heartbeat heartbeat;
    heartbeat.set_module(module);
    heartbeat.set_sequence(sequence);
    heartbeat.set_stamp_ns(stamp_ns);
    heartbeat.set_state(navigation::lifecycle::MODULE_STATE_READY);
    return heartbeat;
}

}  // namespace

int main()
{
    navigation::config::GuardianConfig configuration;
    configuration.set_heartbeat_timeout_ms(200);
    configuration.set_heartbeat_fatal_ms(400);
    configuration.set_allow_reflex_only(true);

    navigation::guardian::SupervisorImplementation supervisor;
    if (!supervisor.Configure(configuration))
    {
        std::cerr << "configure failed\n";
        return 1;
    }
    const std::int64_t now = 1000000000LL;
    std::vector<navigation::lifecycle::Heartbeat> heartbeats = {
        MakeHeartbeat("navigation", 1, now), MakeHeartbeat("safety", 1, now),
        MakeHeartbeat("behavior", 1, now)};
    if (supervisor.Evaluate(heartbeats, now) !=
        navigation::lifecycle::DEGRADATION_MODE_NORMAL)
    {
        std::cerr << "normal failed\n";
        return 1;
    }
    if (supervisor.Evaluate({}, now + 300000000LL) !=
        navigation::lifecycle::DEGRADATION_MODE_SAFE_STOP)
    {
        std::cerr << "missing safety did not stop\n";
        return 1;
    }
    heartbeats[1] = MakeHeartbeat("safety", 2, now + 300000000LL);
    const auto reflex = supervisor.Evaluate(heartbeats, now + 300000000LL);
    if (reflex != navigation::lifecycle::DEGRADATION_MODE_REFLEX_ONLY)
    {
        std::cerr << "navigation timeout did not degrade\n";
        return 1;
    }
    return 0;
}
