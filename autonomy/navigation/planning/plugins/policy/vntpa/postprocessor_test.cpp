/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>
#include <vector>

#include "navigation/planning/plugins/policy/vntpa/postprocessor.hpp"

int main()
{
    using navigation::ActionChunk;
    using navigation::planning::vntpa::Postprocessor;
    using navigation::planning::vntpa::PostprocessorOptions;

    PostprocessorOptions options;
    options.chunk_length = 4;
    options.dt_s = 0.1;
    options.max_linear_mps = 0.5;
    options.max_angular_radps = 1.0;
    Postprocessor processor(options);

    std::vector<float> actions(12, 2.0F);
    ActionChunk chunk;
    if (!processor.Build(actions, 4, 7, 100, &chunk))
    {
        std::cerr << "build failed\n";
        return 1;
    }
    if (chunk.actions_size() != 4 || std::abs(chunk.dt_s() - 0.1) > 1e-9)
    {
        std::cerr << "shape failed\n";
        return 1;
    }
    if (chunk.actions(0).x() > 0.5 + 1e-9 ||
        chunk.actions(0).theta() > 1.0 + 1e-9)
    {
        std::cerr << "clamp failed\n";
        return 1;
    }
    if (chunk.sequence() != 7 || chunk.produced_at_ns() != 100)
    {
        std::cerr << "metadata failed\n";
        return 1;
    }
    actions[0] = std::nanf("");
    if (processor.Build(actions, 4, 7, 100, &chunk))
    {
        std::cerr << "nan accepted\n";
        return 1;
    }
    return 0;
}
