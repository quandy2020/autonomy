/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/planning/plugins/memory/frame_queue_memory.hpp"
#include "navigation/planning/plugins/memory/single_frame_memory.hpp"

int main()
{
    using namespace navigation;  // NOLINT

    memory::Keyframe keyframe;
    keyframe.pose.set_x(1.0);
    keyframe.embedding_dim = 1;
    keyframe.embedding[0] = 1.0F;

    planning::SingleFrameMemory single;
    if (!single.Insert(keyframe).Ok() || single.Size() != 1)
    {
        std::cerr << "single insert failed\n";
        return 1;
    }
    if (!single.View(common::Pose2D{}).Valid() ||
        single.View(common::Pose2D{}).Size() != 1)
    {
        std::cerr << "single view failed\n";
        return 1;
    }
    single.Reset(false);
    if (single.Size() != 0)
    {
        std::cerr << "single reset failed\n";
        return 1;
    }

    plugins::FrameQueueMemoryOptions options;
    options.set_capacity(2);
    planning::FrameQueueMemory queue(options);
    for (int i = 0; i < 3; ++i)
    {
        keyframe.pose.set_x(static_cast<double>(i));
        queue.Insert(keyframe);
    }
    if (queue.Size() != 2 || queue.View(common::Pose2D{}).Size() != 2)
    {
        std::cerr << "queue capacity failed\n";
        return 1;
    }
    return 0;
}
