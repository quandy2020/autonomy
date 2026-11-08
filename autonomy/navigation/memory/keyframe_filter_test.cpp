/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/memory/keyframe_filter.hpp"

namespace
{

navigation::memory::Keyframe MakeKeyframe(double x, float embedding, std::int64_t stamp_ns)
{
    navigation::memory::Keyframe keyframe;
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
    navigation::memory::KeyframeFilterOptions options;
    options.max_entries = 4;
    options.novelty_threshold_eta = 0.1;
    options.min_interval_s = 0.0;
    options.min_motion = 0.0;
    options.min_motion_rad = 0.0;
    navigation::memory::KeyframeFilter filter(options);

    bool accepted = false;
    if (!filter.TryAccept(MakeKeyframe(0.0, 1.0F, 1000000000LL), &accepted) || !accepted)
    {
        std::cerr << "first keyframe rejected\n";
        return 1;
    }
    if (!filter.TryAccept(MakeKeyframe(0.1, 1.0F, 2000000000LL), &accepted) || accepted)
    {
        std::cerr << "duplicate accepted\n";
        return 1;
    }
    if (!filter.TryAccept(MakeKeyframe(0.2, 0.0F, 3000000000LL), &accepted) || !accepted)
    {
        std::cerr << "novel keyframe rejected\n";
        return 1;
    }
    if (filter.Size() != 2)
    {
        std::cerr << "size mismatch\n";
        return 1;
    }
    filter.Reset();
    if (filter.Size() != 0)
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
