/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file benchmark_main.cpp
 * @brief Latency micro-benchmarks for hot-path components.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "navigation/common/platform_registry.hpp"
#include "navigation/memory/keyframe_filter.hpp"
#include "navigation/memory/voxel_grid.hpp"
#include "navigation/planning/chunk_blender.hpp"
#include "navigation/safety/cbf_filter.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

double Ms(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

void Report(const char* name, std::vector<double> samples)
{
    if (samples.empty())
    {
        return;
    }
    std::sort(samples.begin(), samples.end());
    const std::size_t p50 = samples[samples.size() / 2];
    const std::size_t p99 = samples[std::min(samples.size() - 1,
                                             samples.size() * 99 / 100)];
    std::printf("%-28s p50=%8.3f ms  p99=%8.3f ms  n=%zu\n", name, samples[p50],
                samples[p99], samples.size());
}

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    constexpr int kIterations = 2000;
    std::vector<double> voxel_latency;
    memory::VoxelGrid grid(0.1);
    for (int i = 0; i < kIterations; ++i)
    {
        const auto begin = Clock::now();
        grid.Insert(0.01 * i, 0.02 * i, 0.0, static_cast<std::uint32_t>(i));
        voxel_latency.push_back(Ms(begin, Clock::now()));
    }
    Report("VoxelGrid::Insert", voxel_latency);

    std::vector<double> filter_latency;
    memory::KeyframeFilterOptions filter_options;
    filter_options.max_entries = 5000;
    memory::KeyframeFilter filter(filter_options);
    for (int i = 0; i < kIterations; ++i)
    {
        memory::Keyframe keyframe;
        keyframe.pose.set_x(0.05 * i);
        keyframe.embedding[0] = static_cast<float>(i % 17) / 17.0F;
        keyframe.embedding[1] = 1.0F - keyframe.embedding[0];
        keyframe.embedding_dim = 2;
        keyframe.stamp_ns = static_cast<std::int64_t>(i) * 1000000000LL;
        keyframe.importance = 1.0F;
        bool accepted = false;
        const auto begin = Clock::now();
        (void)filter.TryAccept(keyframe, &accepted);
        filter_latency.push_back(Ms(begin, Clock::now()));
    }
    Report("KeyframeFilter::TryAccept", filter_latency);

    common::PlatformProfile profile;
    profile.max_linear_mps = 0.5;
    profile.max_angular_radps = 1.0;
    profile.max_acceleration_mps2 = 1.0;
    profile.max_jerk_mps3 = 5.0;
    profile.footprint_radius = 0.3;
    safety::CbfFilter cbf;
    cbf.Configure(profile, 2.0, 0.05);
    ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(1);
    for (int i = 0; i < 16; ++i)
    {
        auto* obstacle = observation.add_obstacles();
        obstacle->set_x(1.0 + 0.2 * (i % 4));
        obstacle->set_y(-0.5 + 0.1 * i);
        obstacle->set_radius(0.05);
    }
    std::vector<double> cbf_latency;
    for (int i = 0; i < kIterations; ++i)
    {
        const auto begin = Clock::now();
        (void)cbf.Project(navigation::common::MakeTwist2D(0.4, 0.0, 0.1), observation);
        cbf_latency.push_back(Ms(begin, Clock::now()));
    }
    Report("CbfFilter::Project", cbf_latency);

    planning::ChunkBlender blender(planning::ChunkBlender::Options{8, 2, 0.1, 2.0, 0, 0});
    ActionChunk chunk;
    chunk.set_dt_s(0.1);
    for (int i = 0; i < 8; ++i)
    {
        chunk.add_actions()->set_x(0.3);
    }
    blender.Push(chunk, 0);
    std::vector<double> blender_latency;
    common::Twist2D command;
    for (int i = 0; i < kIterations; ++i)
    {
        if (!blender.HasActions())
        {
            blender.Push(chunk, 0);
        }
        const auto begin = Clock::now();
        (void)blender.Consume(0.066, &command);
        blender_latency.push_back(Ms(begin, Clock::now()));
    }
    Report("ChunkBlender::Consume", blender_latency);
    return 0;
}
