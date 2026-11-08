/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <vector>

#include "navigation/memory/loop_aligner.hpp"

int main()
{
    navigation::memory::Keyframe query;
    query.id = 1;
    query.pose.set_x(1.0);
    query.embedding[0] = 1.0F;
    query.embedding_dim = 2;

    navigation::memory::Keyframe candidate = query;
    candidate.id = 2;
    candidate.pose.set_x(1.2);
    std::vector<navigation::memory::Keyframe> memory{candidate};

    navigation::memory::LoopAligner aligner(0.9, 2.0);
    const auto match = aligner.Find(query, memory);
    if (!match.has_value() || match->id != 2 || match->similarity < 0.999 ||
        std::abs(match->correction.x() - 0.2) > 1e-9)
    {
        std::cerr << "loop alignment failed\n";
        return 1;
    }
    candidate.embedding[0] = -1.0F;
    memory[0] = candidate;
    if (aligner.Find(query, memory).has_value())
    {
        std::cerr << "dissimilar candidate accepted\n";
        return 1;
    }
    return 0;
}
