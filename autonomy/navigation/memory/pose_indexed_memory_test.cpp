/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/plugins/memory/pose_indexed_memory.hpp"

namespace
{

navigation::memory::Keyframe MakeKeyframe(std::uint64_t id, double x, float embedding,
                                         std::int64_t stamp_ns)
{
    navigation::memory::Keyframe keyframe;
    keyframe.id = id;
    keyframe.pose.set_x(x);
    keyframe.embedding[0] = embedding;
    keyframe.embedding[1] = 1.0F - embedding;
    keyframe.embedding_dim = 2;
    keyframe.stamp_ns = stamp_ns;
    keyframe.importance = 1.0F;
    return keyframe;
}

}  // namespace

int main()
{
    navigation::plugins::PoseIndexedMemoryOptions options;
    options.set_max_entries(10);
    options.set_novelty_threshold_eta(0.1);
    options.set_min_insert_interval_s(0.0);
    options.set_query_radius(10.0);
    navigation::planning::PoseIndexedMemory memory(options);

    if (!memory.Insert(MakeKeyframe(1, 0.0, 1.0F, 1000000000LL)).Ok())
    {
        std::cerr << "insert failed\n";
        return 1;
    }
    memory.Insert(MakeKeyframe(2, 1.0, 0.0F, 2000000000LL));

    navigation::common::Pose2D pose;
    const auto view = memory.View(pose);
    if (!view.Valid() || view.Size() != 2)
    {
        std::cerr << "view failed\n";
        return 1;
    }
    const auto report = memory.CheckConsistency(pose, 1);
    if (!report.compatible() || report.code() != navigation::CONSISTENCY_CODE_OK)
    {
        std::cerr << "consistency failed\n";
        return 1;
    }
    memory.Reset(false);
    if (memory.Size() != 0)
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
