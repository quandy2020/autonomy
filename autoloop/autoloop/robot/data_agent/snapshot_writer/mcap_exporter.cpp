/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file mcap_exporter.cpp
 * @brief Tiny MCAP: magic + opcode records.
 */

#include "robot/data_agent/snapshot_writer/mcap_exporter.hpp"

#include <cstdint>
#include <fstream>

#include "common/base/path_util.hpp"

namespace autoloop {
namespace robot {
namespace {

void PutU64(std::string* o, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        o->push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    }
}

void PutU32(std::string* o, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        o->push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    }
}

void PutStr(std::string* o, const std::string& s) {
    PutU32(o, static_cast<uint32_t>(s.size()));
    o->append(s);
}

}  // namespace

bool ExportMcap(const std::string& path, const std::vector<Sample>& samples) {
    // MCAP magic (Foxglove).
    static const char kMagic[] = "\x89MCAP0\r\n";
    std::string blob;
    blob.append(kMagic, 8);
    for (const auto& s : samples) {
        blob.push_back(0x05);  // Message opcode (local dialect)
        std::string rec;
        PutStr(&rec, s.channel);
        PutStr(&rec, s.message_type);
        PutU64(&rec, static_cast<uint64_t>(s.clock.mono_ns()));
        PutU64(&rec, static_cast<uint64_t>(s.clock.utc_ns()));
        PutU32(&rec, s.sequence);
        PutU32(&rec, s.flags);
        PutStr(&rec, s.payload);
        PutU64(&blob, rec.size());
        blob.append(rec);
    }
    return common::WriteFile(path, blob);
}

}  // namespace robot
}  // namespace autoloop
