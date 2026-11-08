/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <vector>

#include "navigation/planning/policy_arbiter.hpp"

int main()
{
    using navigation::planning::PolicyArbiter;
    using navigation::planning::PolicyCandidate;

    std::vector<PolicyCandidate> candidates = {
        {"vntpa", true, 1, 0.8, 10}, {"vint", true, 2, 0.5, 20}};
    if (PolicyArbiter::Select(candidates) != "vint")
    {
        std::cerr << "priority selection failed\n";
        return 1;
    }
    candidates[1].healthy = false;
    if (PolicyArbiter::Select(candidates) != "vntpa")
    {
        std::cerr << "health selection failed\n";
        return 1;
    }
    candidates.clear();
    if (!PolicyArbiter::Select(candidates).empty())
    {
        std::cerr << "empty selection failed\n";
        return 1;
    }
    return 0;
}
