/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/memory/spatial_index.hpp"

int main()
{
    navigation::memory::SpatialIndex index(0.1);
    navigation::common::Pose2D pose;
    index.Insert(11, pose);
    pose.set_x(1.0);
    index.Insert(22, pose);
    if (index.Size() != 2)
    {
        std::cerr << "insert failed\n";
        return 1;
    }
    pose.set_x(0.0);
    const auto near = index.QueryRadius(pose, 0.5);
    if (near.size() != 1 || near[0] != 11)
    {
        std::cerr << "radius query failed\n";
        return 1;
    }
    index.Clear();
    if (index.Size() != 0)
    {
        std::cerr << "clear failed\n";
        return 1;
    }
    return 0;
}
