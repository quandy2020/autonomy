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
    // A new horizon may reuse storage, but consumed motion must not return
    // when a zero chunk is played or extended into those old slots.
    auto stopped = MakeChunk(8, 0.0);
    for (auto& action : *stopped.mutable_actions()) action.set_theta(0.0);
    if (!blender.Push(stopped, 2)) return 1;
    for (int i = 0; i < 7; ++i) {
        if (!blender.Consume(0.1, &command) ||
            !Close(command.x(), 0.0) || !Close(command.theta(), 0.0)) {
            std::cerr << "consumed motion revived in a new zero horizon\n";
            return 1;
        }
    }
    auto short_stop = stopped;
    short_stop.mutable_actions()->DeleteSubrange(4, 4);
    if (!blender.Push(short_stop, 3) || !blender.Consume(0.2, &command) ||
        !blender.Push(short_stop, 4)) return 1;
    while (blender.Consume(0.1, &command)) {
        if (!Close(command.x(), 0.0) || !Close(command.theta(), 0.0)) {
            std::cerr << "extending a zero horizon reused stale tail slots\n";
            return 1;
        }
    }
    // The last slot is not consumable without its successor. At exact
    // capacity, an exhausted index must allow the next horizon to restart.
    ChunkBlender capacity(ChunkBlender::Options{});
    if (!capacity.Push(MakeChunk(32, 1.0), 0)) return 1;
    for (int i = 0; i < 31; ++i) {
        if (!capacity.Consume(0.1, &command)) return 1;
    }
    if (capacity.HasActions() || !capacity.Push(stopped, 1) ||
        !capacity.Consume(0.1, &command) || !Close(command.x(), 0.0)) {
        std::cerr << "exhausted capacity blocked the next horizon\n";
        return 1;
    }
    // Clearing stale slots must keep the previous emitted command used by
    // configured acceleration limits across horizon boundaries.
    ChunkBlender limited(ChunkBlender::Options{8, 2, 0.1, 2.0, 0.5, 0.0});
    if (!limited.Push(MakeChunk(8, 1.0), 0)) return 1;
    while (limited.Consume(0.1, &command)) {}
    if (!limited.Push(stopped, 1) || !limited.Consume(0.1, &command) ||
        std::abs(command.x() - 0.95) > 1e-5) {
        std::cerr << "new horizon bypassed acceleration continuity\n";
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
