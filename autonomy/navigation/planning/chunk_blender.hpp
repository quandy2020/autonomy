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
 * @file chunk_blender.hpp
 * @brief Time-based action chunk blending with acceleration continuity.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "navigation/common/twist2d.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{

/**
 * @brief Consumes fixed-length action chunks at the control rate.
 *
 * @details
 * A chunk has @c action_dt_s spacing, while Runner ticks at a different rate.
 * Consume advances a fractional position and interpolates between actions, so
 * the commanded speed does not scale with the tick rate. New chunks are fused
 * into the remaining slots with a decaying weight, and the emitted command is
 * acceleration-limited when limits are configured.
 *
 * @par Threading
 * Single-threaded; owned by Runner.
 */
class ChunkBlender
{
public:
    /** @brief Blending and limit options. */
    struct Options
    {
        /** @brief Actions per chunk. */
        std::size_t chunk_length{8};
        /** @brief Actions guaranteed to execute before a new chunk. */
        std::size_t execute_length{2};
        /** @brief Seconds per action. */
        double action_dt_s{0.1};
        /** @brief Blend decay time constant in seconds. */
        double blend_tau{2.0};
        /** @brief Acceleration limit; 0 disables. */
        double max_acceleration_mps2{0.0};
        /** @brief Jerk limit; 0 disables. */
        double max_jerk_mps3{0.0};
    };

    /** @brief Construct with blend/lookahead options. */
    explicit ChunkBlender(Options options);

    /**
     * @brief Fuse a new chunk into the remaining horizon.
     * @return false when the chunk is empty or malformed.
     */
    bool Push(const ActionChunk& chunk, std::int64_t now_ns);

    /**
     * @brief Sample the next command.
     * @param[in] dt_s Control period.
     * @param[out] command Interpolated, optionally limited command.
     * @return false when no action remains.
     */
    bool Consume(double dt_s, common::Twist2D* command);

    /** @brief Drop all queued actions. */
    void Reset();

    /** @brief True when a chunk is active and not exhausted. */
    [[nodiscard]] bool HasActions() const;

    /** @brief Current fractional position in action steps. */
    [[nodiscard]] double Position() const;

    /** @brief Number of slots occupied by the current chunk. */
    [[nodiscard]] std::size_t ChunkLength() const;

private:
    /** @brief Fixed slot capacity for one chunk. */
    static constexpr std::size_t kMaxChunkLength = 32;

    /** @brief One action slot with blend weight. */
    struct Slot
    {
        /** @brief Action value. */
        common::Twist2D action;
        /** @brief Blend weight in [0,1]. */
        double weight{0.0};
        /** @brief True when the slot holds an action. */
        bool valid{false};
    };

    /** @brief Blend/lookahead options. */
    Options options_;
    /** @brief Active chunk slots. */
    std::array<Slot, kMaxChunkLength> slots_{};
    /** @brief Valid slots in the active chunk. */
    std::size_t chunk_length_{0};
    /** @brief Fractional playback position in slots. */
    double position_{0.0};
    /** @brief Seconds represented by one slot. */
    double action_dt_s_{0.1};
    /** @brief Time of the last chunk push. */
    std::int64_t last_push_ns_{0};
    /** @brief Last emitted command for transition blending. */
    common::Twist2D last_command_{};
    /** @brief True once last_command_ is valid. */
    bool has_last_command_{false};
};

}  // namespace planning
}  // namespace navigation
