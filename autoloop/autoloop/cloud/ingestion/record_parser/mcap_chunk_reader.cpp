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

#include "cloud/ingestion/record_parser/mcap_chunk_reader.hpp"
#include "common/base/path_util.hpp"

namespace autoloop {
namespace cloud {
namespace {
uint32_t U32(const std::string& b, size_t i) {
    uint32_t v = 0;
    for (int k = 0; k < 4; ++k) v |= static_cast<uint32_t>(static_cast<unsigned char>(b[i + k])) << (8 * k);
    return v;
}
uint64_t U64(const std::string& b, size_t i) {
    uint64_t v = 0;
    for (int k = 0; k < 8; ++k) v |= static_cast<uint64_t>(static_cast<unsigned char>(b[i + k])) << (8 * k);
    return v;
}
bool GetStr(const std::string& b, size_t* i, std::string* o) {
    if (*i + 4 > b.size()) return false;
    const uint32_t n = U32(b, *i);
    *i += 4;
    if (*i + n > b.size()) return false;
    o->assign(b.data() + *i, n);
    *i += n;
    return true;
}
}  // namespace

bool ReadMcapSamples(const std::string& path, std::vector<robot::Sample>* out) {
    if (out == nullptr) return false;
    std::string b;
    if (!common::ReadFile(path, &b) || b.size() < 8) return false;
    size_t i = 8;
    while (i + 9 <= b.size()) {
        const uint8_t op = static_cast<uint8_t>(b[i]);
        ++i;
        const uint64_t n = U64(b, i);
        i += 8;
        if (i + n > b.size()) return false;
        if (op == 0x05) {
            size_t j = 0;
            const std::string rec = b.substr(i, static_cast<size_t>(n));
            robot::Sample s;
            if (!GetStr(rec, &j, &s.channel)) return false;
            if (!GetStr(rec, &j, &s.message_type)) return false;
            if (j + 16 + 8 > rec.size()) return false;
            s.clock.set_mono_ns(static_cast<int64_t>(U64(rec, j))); j += 8;
            s.clock.set_utc_ns(static_cast<int64_t>(U64(rec, j))); j += 8;
            s.sequence = U32(rec, j); j += 4;
            s.flags = static_cast<uint16_t>(U32(rec, j)); j += 4;
            if (!GetStr(rec, &j, &s.payload)) return false;
            out->push_back(std::move(s));
        }
        i += static_cast<size_t>(n);
    }
    return true;
}

}  // namespace cloud
}  // namespace autoloop
