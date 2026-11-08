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
 * @file snapshot_serializer.hpp
 * @brief Versioned, checksummed memory snapshot encoding.
 */

#pragma once

#include <string>
#include <vector>

#include "navigation/common/status.hpp"
#include "navigation/memory/context_view.hpp"

namespace navigation
{
namespace memory
{

class SnapshotSerializer
{
public:
    /** @brief On-disk format version. */
    static constexpr std::uint32_t kFormatVersion = 1;

    /**
     * @brief Encode keyframes into a checksummed byte string.
     *
     * @param[in] keyframes Entries to encode.
     * @param[in] epoch Snapshot epoch.
     * @param[out] bytes Encoded payload.
     * @return false on too many entries or invalid dimension.
     */
    bool Serialize(const std::vector<Keyframe>& keyframes, std::uint32_t epoch,
                   std::string* bytes) const;

    /**
     * @brief Decode a snapshot payload.
     *
     * @param[in] bytes Encoded payload.
     * @param[out] keyframes Decoded entries.
     * @param[out] epoch Decoded epoch.
     * @return false on magic/version/CRC/shape mismatch.
     */
    bool Deserialize(const std::string& bytes, std::vector<Keyframe>* keyframes,
                     std::uint32_t* epoch) const;

    /**
     * @brief Serialize and write atomically.
     *
     * @param[in] path Destination file.
     * @param[in] keyframes Entries to encode.
     * @param[in] epoch Snapshot epoch.
     */
    bool Save(const std::string& path, const std::vector<Keyframe>& keyframes,
              std::uint32_t epoch) const;

    /**
     * @brief Read and decode a snapshot file.
     *
     * @param[in] path Source file.
     * @param[out] keyframes Decoded entries.
     * @param[out] epoch Decoded epoch.
     */
    bool Load(const std::string& path, std::vector<Keyframe>* keyframes,
              std::uint32_t* epoch) const;
};

}  // namespace memory
}  // namespace navigation
