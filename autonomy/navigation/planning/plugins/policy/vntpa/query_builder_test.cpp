/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/plugins/policy/vntpa/query_builder.hpp"

int main()
{
    using navigation::GoalSpec;
    using navigation::common::Pose2D;
    using navigation::planning::vntpa::QueryBuilder;

    QueryBuilder builder;
    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    auto* pose = goal.mutable_pose();
    pose->mutable_pose()->mutable_position()->set_x(1.0);
    pose->mutable_pose()->mutable_orientation()->set_w(1.0);

    Pose2D current;
    current.set_x(2.0);
    current.set_y(3.0);
    current.set_theta(0.5);
    std::vector<float> query;
    std::vector<float> goal_xy;
    bool has_goal = false;
    if (!builder.Build(current, goal, &query, &goal_xy, &has_goal) ||
        !has_goal || query.size() != 3 || goal_xy.size() != 2 ||
        std::abs(query[0] - 2.0F) > 1e-6F ||
        std::abs(query[2] - 0.5F) > 1e-6F ||
        std::abs(goal_xy[0] - 1.0F) > 1e-6F)
    {
        std::cerr << "build failed\n";
        return 1;
    }
    GoalSpec empty;
    if (!builder.Build(current, empty, &query, &goal_xy, &has_goal) || has_goal ||
        query.size() != 3 || goal_xy.size() != 2)
    {
        std::cerr << "empty goal failed\n";
        return 1;
    }
    return 0;
}
