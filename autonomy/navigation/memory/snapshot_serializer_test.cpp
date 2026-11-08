/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <string>
#include <vector>

#include "navigation/memory/snapshot_serializer.hpp"

int main()
{
    navigation::memory::Keyframe keyframe;
    keyframe.id = 7;
    keyframe.pose.set_x(1.5);
    keyframe.embedding[0] = 0.25F;
    keyframe.embedding[1] = 0.75F;
    keyframe.embedding_dim = 2;
    keyframe.stamp_ns = 42;
    keyframe.epoch = 3;
    keyframe.observation_count = 5;
    std::vector<navigation::memory::Keyframe> keyframes{keyframe};

    navigation::memory::SnapshotSerializer serializer;
    std::string bytes;
    if (!serializer.Serialize(keyframes, 3, &bytes) || bytes.empty())
    {
        std::cerr << "serialize failed\n";
        return 1;
    }
    std::vector<navigation::memory::Keyframe> decoded;
    std::uint32_t epoch = 0;
    if (!serializer.Deserialize(bytes, &decoded, &epoch) || epoch != 3 ||
        decoded.size() != 1 || decoded[0].id != 7 || decoded[0].embedding_dim != 2)
    {
        std::cerr << "deserialize failed\n";
        return 1;
    }
    const std::string path = "/tmp/navigation_snapshot_test.bin";
    if (!serializer.Save(path, keyframes, 3) || !serializer.Load(path, &decoded, &epoch) ||
        decoded.size() != 1)
    {
        std::cerr << "file round trip failed\n";
        return 1;
    }
    std::string corrupted = bytes;
    corrupted[corrupted.size() / 2] ^= 0xFF;
    if (serializer.Deserialize(corrupted, &decoded, &epoch))
    {
        std::cerr << "corruption accepted\n";
        return 1;
    }
    return 0;
}
