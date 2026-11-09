/* Copyright 2026 The Navigation Authors (navigation) */
#include <cstring>
#include <iostream>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/coverage_gate.hpp"

int main()
{
    navigation::observation::Observation observation;
    auto* depth = observation.mutable_depth();
    depth->set_width(32);
    depth->set_height(32);
    depth->set_encoding("32FC1");
    std::vector<float> values(32 * 32, 1.5F);
    depth->set_data(reinterpret_cast<const char*>(values.data()),
                    values.size() * sizeof(float));

    navigation::common::Pose2D pose;
    pose.set_theta(0.0);
    const auto frame = navigation::memory::FrameCells(observation, pose);
    if (frame.patches == 0 || frame.claims.empty())
    {
        std::cerr << "expected coverage patches from a filled depth image\n";
        return 1;
    }

    navigation::memory::CoverageGate gate(1);
    if (!gate.Offer(frame, true))
    {
        std::cerr << "first frame was not forced\n";
        return 1;
    }
    if (gate.Offer(frame, false))
    {
        std::cerr << "duplicate frame should not pass the coverage gate\n";
        return 1;
    }
    return 0;
}
