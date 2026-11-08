/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/chunk_blender.hpp"

namespace
{

navigation::ActionChunk MakeChunk(std::size_t count, double linear, double dt = 0.1)
{
    navigation::ActionChunk chunk;
    chunk.set_dt_s(dt);
    for (std::size_t i = 0; i < count; ++i)
    {
        auto* action = chunk.add_actions();
        action->set_x(linear);
        action->set_theta(0.1);
    }
    return chunk;
}

bool Close(double lhs, double rhs, double epsilon = 1e-9)
{
    return std::abs(lhs - rhs) <= epsilon;
}

}  // namespace

int main()
{
    using navigation::common::Twist2D;
    using navigation::planning::ChunkBlender;

    ChunkBlender blender(ChunkBlender::Options{8, 2, 0.1, 2.0, 0.0, 0.0});
    if (blender.Consume(0.1, nullptr))
    {
        std::cerr << "empty blender consumed\n";
        return 1;
    }
    if (!blender.Push(MakeChunk(8, 1.0), 0))
    {
        std::cerr << "push failed\n";
        return 1;
    }
    Twist2D command;
    if (!blender.Consume(0.1, &command) || !Close(command.x(), 1.0))
    {
        std::cerr << "first consume failed\n";
        return 1;
    }
    // 15 Hz ticks with 10 Hz actions must interpolate, not advance one action
    // per tick.
    if (!blender.Consume(0.0666666667, &command) || !Close(command.x(), 1.0))
    {
        std::cerr << "interpolation failed\n";
        return 1;
    }
    if (blender.Position() < 1.66 || blender.Position() > 1.68)
    {
        std::cerr << "position scaling failed: " << blender.Position() << "\n";
        return 1;
    }

    if (!blender.Push(MakeChunk(8, 0.0), 1))
    {
        std::cerr << "second push failed\n";
        return 1;
    }
    if (!blender.Consume(0.1, &command))
    {
        std::cerr << "consume after second push failed\n";
        return 1;
    }
    if (!std::isfinite(command.x()) || command.x() < 0.0 ||
        command.x() > 1.0)
    {
        std::cerr << "blend out of range\n";
        return 1;
    }

    for (int i = 0; i < 16; ++i)
    {
        blender.Consume(0.1, &command);
    }
    if (blender.HasActions())
    {
        std::cerr << "chunk not exhausted\n";
        return 1;
    }
    if (blender.Consume(0.1, &command))
    {
        std::cerr << "exhausted chunk consumed\n";
        return 1;
    }
    blender.Reset();
    if (blender.HasActions() || blender.ChunkLength() != 0)
    {
        std::cerr << "reset failed\n";
        return 1;
    }
    return 0;
}
