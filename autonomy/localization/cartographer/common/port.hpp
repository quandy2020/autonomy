/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <cinttypes>
#include <cmath>
#include <string>

#include <zlib.h>

// Thread-safety annotations (no-op unless compiling with clang).
#if defined(__SUPPORT_TS_ANNOTATION__) || defined(__clang__)
#define CARTOGRAPHER_THREAD_ANNOTATION__(x) __attribute__((x))
#else
#define CARTOGRAPHER_THREAD_ANNOTATION__(x)
#endif

#define GUARDED_BY(x) CARTOGRAPHER_THREAD_ANNOTATION__(guarded_by(x))
#define PT_GUARDED_BY(x) CARTOGRAPHER_THREAD_ANNOTATION__(pt_guarded_by(x))
#define EXCLUSIVE_LOCKS_REQUIRED(...) \
    CARTOGRAPHER_THREAD_ANNOTATION__(exclusive_locks_required(__VA_ARGS__))
#define LOCKS_EXCLUDED(...) CARTOGRAPHER_THREAD_ANNOTATION__(locks_excluded(__VA_ARGS__))
#define REQUIRES(...) CARTOGRAPHER_THREAD_ANNOTATION__(requires_capability(__VA_ARGS__))

namespace cartographer {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common {

inline int RoundToInt(const float x) {
    return std::lround(x);
}

inline int RoundToInt(const double x) {
    return std::lround(x);
}

inline int64 RoundToInt64(const float x) {
    return std::lround(x);
}

inline int64 RoundToInt64(const double x) {
    return std::lround(x);
}

inline void FastGzipString(const std::string& uncompressed, std::string* compressed) {
    compressed->clear();
    z_stream stream{};
    if (deflateInit2(&stream, Z_BEST_SPEED, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY) != Z_OK) {
        return;
    }
    stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(uncompressed.data()));
    stream.avail_in = static_cast<uInt>(uncompressed.size());
    compressed->resize(deflateBound(&stream, static_cast<uLong>(uncompressed.size())));
    stream.next_out = reinterpret_cast<Bytef*>(compressed->data());
    stream.avail_out = static_cast<uInt>(compressed->size());
    const int ret = deflate(&stream, Z_FINISH);
    if (ret == Z_STREAM_END) {
        compressed->resize(stream.total_out);
    } else {
        compressed->clear();
    }
    deflateEnd(&stream);
}

inline void FastGunzipString(const std::string& compressed, std::string* decompressed) {
    decompressed->clear();
    if (compressed.empty()) {
        return;
    }
    z_stream stream{};
    if (inflateInit2(&stream, 15 + 16) != Z_OK) {
        return;
    }
    stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(compressed.data()));
    stream.avail_in = static_cast<uInt>(compressed.size());
    int ret = Z_OK;
    while (ret == Z_OK) {
        if (stream.total_out >= decompressed->size()) {
            decompressed->resize(decompressed->size() + compressed.size() * 2 + 64);
        }
        stream.next_out = reinterpret_cast<Bytef*>(decompressed->data() + stream.total_out);
        stream.avail_out = static_cast<uInt>(decompressed->size() - stream.total_out);
        ret = inflate(&stream, Z_NO_FLUSH);
    }
    if (ret == Z_STREAM_END) {
        decompressed->resize(stream.total_out);
    } else {
        decompressed->clear();
    }
    inflateEnd(&stream);
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
