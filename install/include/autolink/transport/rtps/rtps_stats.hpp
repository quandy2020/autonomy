/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <atomic>
#include <cstdint>
#include <sstream>
#include <string>

namespace autolink {
namespace transport {

/**
 * Process-wide RTPS counters (atomic). No Fast DDS dependency — usable in
 * unit tests without AUTOLINK_ENABLE_FASTDDS.
 */
class RtpsStats {
public:
    static RtpsStats& Instance() {
        static RtpsStats inst;
        return inst;
    }

    void AddSent(uint64_t n = 1) {
        sent_.fetch_add(n, std::memory_order_relaxed);
    }
    void AddRecv(uint64_t n = 1) {
        recv_.fetch_add(n, std::memory_order_relaxed);
    }
    void AddWriteFail(uint64_t n = 1) {
        write_fail_.fetch_add(n, std::memory_order_relaxed);
    }
    void AddOversize(uint64_t n = 1) {
        oversize_.fetch_add(n, std::memory_order_relaxed);
    }

    void SetMatchedReaders(int64_t v) {
        matched_readers_.store(v, std::memory_order_relaxed);
    }
    void SetMatchedWriters(int64_t v) {
        matched_writers_.store(v, std::memory_order_relaxed);
    }

    uint64_t sent() const {
        return sent_.load(std::memory_order_relaxed);
    }
    uint64_t recv() const {
        return recv_.load(std::memory_order_relaxed);
    }
    uint64_t write_fail() const {
        return write_fail_.load(std::memory_order_relaxed);
    }
    uint64_t oversize() const {
        return oversize_.load(std::memory_order_relaxed);
    }
    int64_t matched_readers() const {
        return matched_readers_.load(std::memory_order_relaxed);
    }
    int64_t matched_writers() const {
        return matched_writers_.load(std::memory_order_relaxed);
    }

    std::string Dump() const {
        std::ostringstream oss;
        oss << "RtpsStats{sent=" << sent() << " recv=" << recv()
            << " write_fail=" << write_fail() << " oversize=" << oversize()
            << " matched_readers=" << matched_readers()
            << " matched_writers=" << matched_writers() << "}";
        return oss.str();
    }

    void Reset() {
        sent_.store(0, std::memory_order_relaxed);
        recv_.store(0, std::memory_order_relaxed);
        write_fail_.store(0, std::memory_order_relaxed);
        oversize_.store(0, std::memory_order_relaxed);
        matched_readers_.store(0, std::memory_order_relaxed);
        matched_writers_.store(0, std::memory_order_relaxed);
    }

private:
    RtpsStats() = default;

    std::atomic<uint64_t> sent_{0};
    std::atomic<uint64_t> recv_{0};
    std::atomic<uint64_t> write_fail_{0};
    std::atomic<uint64_t> oversize_{0};
    std::atomic<int64_t> matched_readers_{0};
    std::atomic<int64_t> matched_writers_{0};
};

}  // namespace transport
}  // namespace autolink
