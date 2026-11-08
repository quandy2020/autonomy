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

#include "navigation/memory/snapshot_serializer.hpp"

#include <cstring>
#include <fstream>
#include <iterator>

namespace navigation
{
namespace memory
{
namespace
{

/** @brief Read all bytes without throwing across the public API. */
bool ReadAllBytes(const std::string& path, std::string* bytes)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        return false;
    }
    bytes->assign(std::istreambuf_iterator<char>(stream),
                  std::istreambuf_iterator<char>());
    return stream.good() || stream.eof();
}

/** @brief Write all bytes without throwing across the public API. */
bool WriteAllBytes(const std::string& path, const std::string& bytes)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream)
    {
        return false;
    }
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(stream);
}

}  // namespace

namespace
{

constexpr char kMagic[8] = {'N', 'A', 'V', 'S', 'N', 'A', 'P', '1'};

std::uint32_t Crc32(const std::string& data)
{
    std::uint32_t crc = 0xFFFFFFFFU;
    for (unsigned char byte : data)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
        {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

template <typename T>
void Append(std::string* bytes, const T& value)
{
    bytes->append(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
bool Read(const std::string& bytes, std::size_t* offset, T* value)
{
    if (*offset + sizeof(T) > bytes.size())
    {
        return false;
    }
    std::memcpy(value, bytes.data() + *offset, sizeof(T));
    *offset += sizeof(T);
    return true;
}

}  // namespace

bool SnapshotSerializer::Serialize(const std::vector<Keyframe>& keyframes,
                                   std::uint32_t epoch, std::string* bytes) const
{
    if (bytes == nullptr || keyframes.size() > kMaxContextEntries)
    {
        return false;
    }
    bytes->clear();
    bytes->append(kMagic, sizeof(kMagic));
    Append(bytes, kFormatVersion);
    Append(bytes, epoch);
    const std::uint32_t count = static_cast<std::uint32_t>(keyframes.size());
    Append(bytes, count);
    for (const auto& keyframe : keyframes)
    {
        Append(bytes, keyframe.id);
        Append(bytes, keyframe.pose.x());
        Append(bytes, keyframe.pose.y());
        Append(bytes, keyframe.pose.theta());
        const std::uint32_t dim = keyframe.embedding_dim;
        Append(bytes, dim);
        if (dim > kMaxEmbeddingDim)
        {
            return false;
        }
        bytes->append(reinterpret_cast<const char*>(keyframe.embedding.data()),
                      sizeof(float) * dim);
        Append(bytes, keyframe.novelty);
        Append(bytes, keyframe.importance);
        Append(bytes, keyframe.stamp_ns);
        Append(bytes, keyframe.epoch);
        Append(bytes, keyframe.observation_count);
    }
    Append(bytes, Crc32(*bytes));
    return true;
}

bool SnapshotSerializer::Deserialize(const std::string& bytes,
                                     std::vector<Keyframe>* keyframes,
                                     std::uint32_t* epoch) const
{
    if (keyframes == nullptr || epoch == nullptr ||
        bytes.size() < sizeof(kMagic) + sizeof(std::uint32_t) * 3 + sizeof(std::uint32_t))
    {
        return false;
    }
    if (std::memcmp(bytes.data(), kMagic, sizeof(kMagic)) != 0)
    {
        return false;
    }
    const std::size_t crc_offset = bytes.size() - sizeof(std::uint32_t);
    std::uint32_t stored_crc = 0;
    std::memcpy(&stored_crc, bytes.data() + crc_offset, sizeof(stored_crc));
    if (stored_crc != Crc32(bytes.substr(0, crc_offset)))
    {
        return false;
    }

    std::size_t offset = sizeof(kMagic);
    std::uint32_t version = 0;
    std::uint32_t read_epoch = 0;
    std::uint32_t count = 0;
    if (!Read(bytes, &offset, &version) || version != kFormatVersion ||
        !Read(bytes, &offset, &read_epoch) || !Read(bytes, &offset, &count) ||
        count > kMaxContextEntries)
    {
        return false;
    }

    std::vector<Keyframe> parsed;
    parsed.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        Keyframe keyframe;
        std::uint32_t dim = 0;
        double x = 0.0;
        double y = 0.0;
        double theta_rad = 0.0;
        if (!Read(bytes, &offset, &keyframe.id) ||
            !Read(bytes, &offset, &x) || !Read(bytes, &offset, &y) ||
            !Read(bytes, &offset, &theta_rad) ||
            !Read(bytes, &offset, &dim) || dim > kMaxEmbeddingDim)
        {
            return false;
        }
        keyframe.pose.set_x(x);
        keyframe.pose.set_y(y);
        keyframe.pose.set_theta(theta_rad);
        if (offset + sizeof(float) * dim > crc_offset)
        {
            return false;
        }
        std::memcpy(keyframe.embedding.data(), bytes.data() + offset,
                    sizeof(float) * dim);
        offset += sizeof(float) * dim;
        keyframe.embedding_dim = dim;
        if (!Read(bytes, &offset, &keyframe.novelty) ||
            !Read(bytes, &offset, &keyframe.importance) ||
            !Read(bytes, &offset, &keyframe.stamp_ns) ||
            !Read(bytes, &offset, &keyframe.epoch) ||
            !Read(bytes, &offset, &keyframe.observation_count))
        {
            return false;
        }
        parsed.push_back(keyframe);
    }
    if (offset != crc_offset)
    {
        return false;
    }
    *keyframes = std::move(parsed);
    *epoch = read_epoch;
    return true;
}

bool SnapshotSerializer::Save(const std::string& path,
                              const std::vector<Keyframe>& keyframes,
                              std::uint32_t epoch) const
{
    std::string bytes;
    if (!Serialize(keyframes, epoch, &bytes))
    {
        return false;
    }
    return WriteAllBytes(path, bytes);
}

bool SnapshotSerializer::Load(const std::string& path,
                              std::vector<Keyframe>* keyframes,
                              std::uint32_t* epoch) const
{
    std::string bytes;
    if (!ReadAllBytes(path, &bytes))
    {
        return false;
    }
    return Deserialize(bytes, keyframes, epoch);
}

}  // namespace memory
}  // namespace navigation
