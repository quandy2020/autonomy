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
 * @file context_view.hpp
 * @brief Immutable keyframe batch handed to the policy worker.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace memory
{

inline constexpr std::size_t kMaxEmbeddingDim = 512;
inline constexpr std::size_t kMaxContextEntries = 64;
inline constexpr std::size_t kMaxTokenCount = 256;
inline constexpr std::size_t kMaxTokenDim = 384;

/**
 * @brief One pose-indexed keyframe.
 *
 * @par Threading
 * Plain value; copied into immutable batches.
 */
struct Keyframe
{
    /** @brief Stable keyframe id. */
    std::uint64_t id{0};
    /** @brief Map-frame keyframe pose. */
    common::Pose2D pose;
    /** @brief Fixed-capacity pooling embedding (loop closure/debug). */
    std::array<float, kMaxEmbeddingDim> embedding{};
    /** @brief Number of valid embedding dimensions. */
    std::uint32_t embedding_dim{0};
    /** @brief DeFM patch tokens; shared so ContextView copies stay cheap. */
    std::shared_ptr<const std::vector<float>> tokens;
    /** @brief Number of patch tokens. */
    std::uint32_t token_count{0};
    /** @brief Patch-token dimension. */
    std::uint32_t token_dim{0};
    /** @brief Novelty at insertion, in [0,1]. */
    float novelty{0.0F};
    /** @brief Eviction importance at insertion. */
    float importance{0.0F};
    /** @brief Observation timestamp in nanoseconds. */
    std::int64_t stamp_ns{0};
    /** @brief Session epoch at insertion. */
    std::uint32_t epoch{0};
    /** @brief Number of observations merged into this keyframe. */
    std::uint32_t observation_count{0};
};

/** @brief Fixed-capacity immutable batch shared with the policy worker. */
struct KeyframeBatch
{
    /** @brief Fixed-capacity entries; only [0,size) are valid. */
    std::array<Keyframe, kMaxContextEntries> entries{};
    /** @brief Number of valid entries. */
    std::size_t size{0};
    /** @brief Epoch all entries belong to. */
    std::uint32_t epoch{0};
    /** @brief Serialization format version. */
    std::uint32_t format_version{1};
};

/**
 * @brief Non-owning handle to an immutable KeyframeBatch.
 *
 * @par Threading
 * Copyable across threads; the underlying batch is never mutated.
 */
class ContextView
{
public:
    /** @brief Construct an invalid (empty) view. */
    ContextView() = default;

    /**
     * @brief Construct from an immutable batch.
     *
     * @param[in] batch Shared batch; ownership co-owned.
     */
    explicit ContextView(std::shared_ptr<const KeyframeBatch> batch)
        : batch_(std::move(batch))
    {
    }

    /** @brief True when a batch is attached. */
    [[nodiscard]] bool Valid() const
    {
        return batch_ != nullptr;
    }

    /** @brief Number of valid context entries. */
    [[nodiscard]] std::size_t Size() const
    {
        return batch_ != nullptr ? batch_->size : 0;
    }

    /** @brief Access the attached batch; callers must check Valid(). */
    [[nodiscard]] const KeyframeBatch& Batch() const
    {
        return *batch_;
    }

    /** @brief Batch epoch, or 0 when invalid. */
    [[nodiscard]] std::uint32_t Epoch() const
    {
        return batch_ != nullptr ? batch_->epoch : 0;
    }

private:
    /** @brief Immutable batch handle. */
    std::shared_ptr<const KeyframeBatch> batch_{nullptr};
};

}  // namespace memory
}  // namespace navigation
