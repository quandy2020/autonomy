/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>
#include <vector>

#include "navigation/memory/drift_compensator.hpp"

int main()
{
    navigation::memory::Keyframe keyframe;
    keyframe.pose.set_x(1.0);
    keyframe.pose.set_y(0.0);
    keyframe.epoch = 1;
    std::vector<navigation::memory::Keyframe> keyframes{keyframe};
    std::uint32_t epoch = 1;

    navigation::memory::DriftCompensator compensator;
    navigation::common::Pose2D correction;
    correction.set_x(0.5);
    if (!compensator.ApplyCorrection(correction, &keyframes, &epoch) || epoch != 2 ||
        std::abs(keyframes[0].pose.x() - 1.5) > 1e-9 || keyframes[0].epoch != 2)
    {
        std::cerr << "correction failed\n";
        return 1;
    }
    return 0;
}
