/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>
#include <vector>

#include "navigation/memory/dead_reckoning_bridge.hpp"
#include "navigation/memory/dynamic_filter.hpp"
#include "navigation/memory/importance_scorer.hpp"

int main()
{
    using namespace navigation::common;  // NOLINT

    automsgs::msgs::sensor_msgs::PointCloud cloud;
    cloud.add_points()->set_x(1.0F);
    cloud.add_points()->set_x(2.0F);
    cloud.add_points()->set_x(std::nanf(""));
    auto* confidence = cloud.add_channels();
    confidence->set_name("dynamic_confidence");
    confidence->add_values(0.1F);
    confidence->add_values(0.9F);
    confidence->add_values(0.0F);
    const auto kept = navigation::memory::DynamicFilter(0.6F).Filter(cloud);
    if (kept.points_size() != 1 || kept.channels_size() != 1 ||
        kept.channels(0).values_size() != 1)
    {
        std::cerr << "dynamic filter failed\n";
        return 1;
    }

    navigation::memory::Keyframe near;
    near.pose.set_x(0.5);
    near.novelty = 0.8F;
    near.stamp_ns = 900000000;
    navigation::memory::Keyframe far = near;
    far.pose.set_x(5.0);
    far.stamp_ns = 100000000;
    navigation::memory::ImportanceScorer scorer;
    if (scorer.Score(near, 1000000000, Pose2D{}) <= scorer.Score(far, 1000000000, Pose2D{}))
    {
        std::cerr << "importance score failed\n";
        return 1;
    }

    navigation::memory::DeadReckoningBridge bridge;
    bridge.Reset(Pose2D{});
    if (!bridge.Update(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), 1.0) ||
        std::abs(bridge.Pose().x() - 0.5) > 1e-9 || !bridge.Extrapolated())
    {
        std::cerr << "dead reckoning failed\n";
        return 1;
    }
    if (bridge.Update(navigation::common::MakeTwist2D(0.5, 0.0, 0.0), 0.0))
    {
        std::cerr << "zero dt accepted\n";
        return 1;
    }
    return 0;
}
