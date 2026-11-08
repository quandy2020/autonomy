/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file goal_sender_main.cpp
 * @brief Publish one navigation goal for autosim; no args = explore.
 */

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include "autolink/autolink.hpp"
#include "navigation/proto/navigation.pb.h"

using navigation::GoalSpec;

int main(int argc, char* argv[])
{
    if (!autolink::Init(argv[0]))
    {
        return 1;
    }
    auto node = autolink::CreateNode("navigation_goal_sender");
    auto writer = node->CreateWriter<GoalSpec>("/navigation/goal");
    if (writer == nullptr)
    {
        std::cerr << "failed to create /navigation/goal writer\n";
        return 1;
    }
    GoalSpec goal;
    goal.set_goal_id("sim-goal");
    goal.set_sequence(1);
    if (argc >= 4)
    {
        const double x = std::atof(argv[1]);
        const double y = std::atof(argv[2]);
        const double yaw = std::atof(argv[3]);
        goal.set_type(GoalSpec::TYPE_POSE);
        auto* stamped = goal.mutable_pose();
        stamped->mutable_header()->set_frame_id("map");
        stamped->mutable_pose()->mutable_position()->set_x(x);
        stamped->mutable_pose()->mutable_position()->set_y(y);
        stamped->mutable_pose()->mutable_orientation()->set_z(std::sin(0.5 * yaw));
        stamped->mutable_pose()->mutable_orientation()->set_w(std::cos(0.5 * yaw));
        goal.set_position_tolerance(0.2);
    }
    else
    {
        goal.set_type(GoalSpec::TYPE_EXPLORATION);
    }
    writer->Write(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return 0;
}
