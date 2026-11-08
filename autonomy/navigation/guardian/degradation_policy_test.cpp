/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/guardian/degradation_policy.hpp"

int main()
{
    using navigation::guardian::DegradationPolicy;
    using navigation::guardian::ModuleHealth;
    using navigation::lifecycle::DegradationMode;

    navigation::config::GuardianConfig configuration;
    configuration.set_allow_reflex_only(true);
    DegradationPolicy policy;
    policy.Configure(configuration);

    if (policy.Evaluate(ModuleHealth{}) != navigation::lifecycle::DEGRADATION_MODE_NORMAL)
    {
        std::cerr << "normal failed\n";
        return 1;
    }
    ModuleHealth navigation_down;
    navigation_down.navigation_ok = false;
    if (policy.Evaluate(navigation_down) !=
        navigation::lifecycle::DEGRADATION_MODE_REFLEX_ONLY)
    {
        std::cerr << "reflex only failed\n";
        return 1;
    }
    ModuleHealth safety_down;
    safety_down.safety_ok = false;
    if (policy.Evaluate(safety_down) != navigation::lifecycle::DEGRADATION_MODE_SAFE_STOP)
    {
        std::cerr << "safe stop failed\n";
        return 1;
    }
    return 0;
}
