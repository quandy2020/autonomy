/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/safety/depth_freshness.hpp"

int main() {
    navigation::safety::DepthFreshness freshness;
    constexpr std::int64_t now = 1000000000;
    if (freshness.Fresh(now)) {
        std::cerr << "missing depth accepted\n";
        return 1;
    }
    // A simulation epoch of zero is independent of the monotonic receive clock.
    freshness.Observe(0, now);
    freshness.Observe(0, now + 200000000);
    if (!freshness.Fresh(now + 200000000) || freshness.Fresh(now + 200000001)) {
        std::cerr << "cached depth renewed its lease\n";
        return 1;
    }
    freshness.Observe(50000000, now + 300000000);
    if (!freshness.Fresh(now + 300000000)) {
        std::cerr << "new depth did not recover\n";
        return 1;
    }
    // A restarted simulator can reset its timestamp; a new frame is still
    // fresh.
    freshness.Observe(0, now + 400000000);
    if (!freshness.Fresh(now + 400000000) || freshness.Fresh(now)) {
        std::cerr << "clock reset handling failed\n";
        return 1;
    }
    freshness.Reset();
    if (freshness.Fresh(now + 400000000))
        return 1;
    return 0;
}
