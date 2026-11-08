/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/plugins/policy/vntpa/query_builder.hpp"

int main()
{
    using navigation::GoalSpec;
    using navigation::common::Pose2D;
    using navigation::planning::vntpa::QueryBuilder;

    QueryBuilder builder(8);
    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    auto* pose = goal.mutable_pose();
    pose->mutable_pose()->mutable_position()->set_x(1.0);
    pose->mutable_pose()->mutable_orientation()->set_w(1.0);

    std::vector<float> query;
    bool has_goal = false;
    if (!builder.Build(Pose2D{}, goal, &query, &has_goal) || !has_goal ||
        query.size() != 8)
    {
        std::cerr << "build failed\n";
        return 1;
    }
    if (std::abs(query[0] - std::tanh(1.0F)) > 1e-5)
    {
        std::cerr << "encoding failed: " << query[0] << "\n";
        return 1;
    }
    GoalSpec empty;
    if (!builder.Build(Pose2D{}, empty, &query, &has_goal) || has_goal ||
        query.size() != 8)
    {
        std::cerr << "empty goal failed\n";
        return 1;
    }
    return 0;
}
