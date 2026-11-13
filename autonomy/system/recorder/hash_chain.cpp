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
 * @file hash_chain.cpp
 * @brief L4 SHA-256 append-only SafetyEvent chain.
 */

#include "autonomy/system/recorder/hash_chain.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

constexpr uint32_t kK[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

uint32_t Rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

std::string HashBytes(const uint8_t* data, size_t len) {
    uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                     0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::vector<uint8_t> msg(data, data + len);
    const uint64_t bitlen = static_cast<uint64_t>(len) * 8;
    msg.push_back(0x80);
    while ((msg.size() % 64) != 56) {
        msg.push_back(0);
    }
    for (int i = 7; i >= 0; --i) {
        msg.push_back(static_cast<uint8_t>((bitlen >> (8 * i)) & 0xff));
    }
    for (size_t off = 0; off < msg.size(); off += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(msg[off + i * 4]) << 24) |
                   (static_cast<uint32_t>(msg[off + i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(msg[off + i * 4 + 2]) << 8) |
                   static_cast<uint32_t>(msg[off + i * 4 + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 =
                Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 =
                Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t temp1 = hh + S1 + ch + kK[i] + w[i];
            const uint32_t S0 = Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = S0 + maj;
            hh = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += hh;
    }
    std::string out(32, '\0');
    for (int i = 0; i < 8; ++i) {
        out[i * 4] = static_cast<char>((h[i] >> 24) & 0xff);
        out[i * 4 + 1] = static_cast<char>((h[i] >> 16) & 0xff);
        out[i * 4 + 2] = static_cast<char>((h[i] >> 8) & 0xff);
        out[i * 4 + 3] = static_cast<char>(h[i] & 0xff);
    }
    return out;
}

std::string Hex(const std::string& bytes) {
    static const char* kHex = "0123456789abcdef";
    std::string o;
    o.resize(bytes.size() * 2);
    for (size_t i = 0; i < bytes.size(); ++i) {
        const auto b = static_cast<unsigned char>(bytes[i]);
        o[i * 2] = kHex[b >> 4];
        o[i * 2 + 1] = kHex[b & 0xf];
    }
    return o;
}

}  // namespace

std::string Sha256(const std::string& data) {
    return HashBytes(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

SafetyHashChain::SafetyHashChain(std::string robot_id)
    : robot_id_(std::move(robot_id)), prev_hash_(32, '\0') {}

proto::SafetyEvent SafetyHashChain::Append(const proto::SafetyEvent& event) {
    proto::SafetyEvent sealed = event;
    sealed.set_prev_hash(prev_hash_);
    std::ostringstream body;
    body << sealed.event_id() << '\n'
         << sealed.kind() << '\n'
         << sealed.episode_id() << '\n'
         << sealed.bt_node() << '\n'
         << sealed.clock().monotonic_ns() << '\n'
         << sealed.version().policy_version() << '\n'
         << sealed.version().git_sha() << '\n';
    const std::string digest = Sha256(prev_hash_ + body.str());
    sealed.set_entry_hash(digest);
    prev_hash_ = digest;
    events_.push_back(sealed);
    return sealed;
}

bool SafetyHashChain::Verify(
    const std::vector<proto::SafetyEvent>& events) const {
    std::string prev(32, '\0');
    for (const auto& e : events) {
        if (e.prev_hash() != prev) {
            return false;
        }
        std::ostringstream body;
        body << e.event_id() << '\n'
             << e.kind() << '\n'
             << e.episode_id() << '\n'
             << e.bt_node() << '\n'
             << e.clock().monotonic_ns() << '\n'
             << e.version().policy_version() << '\n'
             << e.version().git_sha() << '\n';
        const std::string digest = Sha256(prev + body.str());
        if (digest != e.entry_hash()) {
            return false;
        }
        prev = digest;
    }
    return true;
}

std::string SafetyHashChain::Seal() const {
    std::ostringstream oss;
    oss << robot_id_ << '\n' << Hex(prev_hash_) << '\n' << events_.size();
    return Hex(Sha256(oss.str()));
}

bool SafetyHashChain::Save(const std::string& path) const {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    for (const auto& e : events_) {
        const std::string bytes = e.SerializeAsString();
        const uint32_t n = static_cast<uint32_t>(bytes.size());
        out.write(reinterpret_cast<const char*>(&n), sizeof(n));
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    return static_cast<bool>(out);
}

bool SafetyHashChain::Load(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::vector<proto::SafetyEvent> loaded;
    while (in) {
        uint32_t n = 0;
        in.read(reinterpret_cast<char*>(&n), sizeof(n));
        if (in.gcount() != sizeof(n)) {
            break;
        }
        std::string bytes(n, '\0');
        in.read(bytes.data(), static_cast<std::streamsize>(n));
        if (static_cast<uint32_t>(in.gcount()) != n) {
            return false;
        }
        proto::SafetyEvent e;
        if (!e.ParseFromString(bytes)) {
            return false;
        }
        loaded.push_back(std::move(e));
    }
    if (!Verify(loaded)) {
        return false;
    }
    events_ = std::move(loaded);
    prev_hash_ = events_.empty() ? std::string(32, '\0')
                                 : events_.back().entry_hash();
    return true;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
