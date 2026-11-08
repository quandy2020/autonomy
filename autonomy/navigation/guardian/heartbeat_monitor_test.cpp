/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/guardian/heartbeat_monitor.hpp"
#include "navigation/proto/lifecycle.pb.h"

int main()
{
    using navigation::guardian::HeartbeatMonitor;
    using navigation::lifecycle::Heartbeat;

    HeartbeatMonitor monitor;
    monitor.Configure(200, 400);

    const std::int64_t now = 1000000000LL;
    Heartbeat heartbeat;
    heartbeat.set_module("navigation");
    heartbeat.set_sequence(1);
    heartbeat.set_stamp_ns(now);
    if (!monitor.Update(heartbeat, now))
    {
        std::cerr << "update failed\n";
        return 1;
    }
    if (!monitor.Check("navigation", now).healthy)
    {
        std::cerr << "fresh heartbeat unhealthy\n";
        return 1;
    }
    const auto stale = monitor.Check("navigation", now + 300000000LL);
    if (stale.healthy || stale.fatal)
    {
        std::cerr << "stale classification failed\n";
        return 1;
    }
    const auto dead = monitor.Check("navigation", now + 500000000LL);
    if (!dead.fatal)
    {
        std::cerr << "fatal classification failed\n";
        return 1;
    }
    if (monitor.Update(heartbeat, now + 600000000LL))
    {
        std::cerr << "stale sequence accepted\n";
        return 1;
    }
    if (!monitor.Check("missing", now).fatal)
    {
        std::cerr << "missing module not fatal\n";
        return 1;
    }
    return 0;
}
