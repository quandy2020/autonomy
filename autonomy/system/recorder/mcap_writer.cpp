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
 * @file mcap_writer.cpp
 * @brief Minimal MCAP write/recover; message payload written without a pack buffer.
 */

#include "autonomy/system/recorder/mcap_writer.hpp"

#include <array>
#include <cstring>
#include <fstream>
#include <ostream>
#include <unordered_map>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

constexpr char kMagic[] = "\x89MCAP0\r\n";

enum Opcode : uint8_t {
    kHeader = 0x01,
    kFooter = 0x02,
    kSchema = 0x03,
    kChannel = 0x04,
    kMessage = 0x05,
    kMetadata = 0x0C,
    kStatistics = 0x0B,
    kDataEnd = 0x0F,
};

void PutU8(std::string* o, uint8_t v) { o->push_back(static_cast<char>(v)); }

void PutU16(std::string* o, uint16_t v) {
    o->push_back(static_cast<char>(v & 0xff));
    o->push_back(static_cast<char>((v >> 8) & 0xff));
}

void PutU32(std::string* o, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        o->push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    }
}

void PutU64(std::string* o, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        o->push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    }
}

void PutBytes(std::string* o, const std::string& s) {
    PutU32(o, static_cast<uint32_t>(s.size()));
    o->append(s);
}

void WriteBytes(std::ostream& o, const char* p, size_t n) {
    o.write(p, static_cast<std::streamsize>(n));
}

void WriteU8(std::ostream& o, uint8_t v) {
    const char c = static_cast<char>(v);
    o.write(&c, 1);
}

void WriteU16(std::ostream& o, uint16_t v) {
    const char b[2] = {static_cast<char>(v & 0xff),
                       static_cast<char>((v >> 8) & 0xff)};
    o.write(b, 2);
}

void WriteU32(std::ostream& o, uint32_t v) {
    char b[4];
    for (int i = 0; i < 4; ++i) {
        b[i] = static_cast<char>((v >> (8 * i)) & 0xff);
    }
    o.write(b, 4);
}

void WriteU64(std::ostream& o, uint64_t v) {
    char b[8];
    for (int i = 0; i < 8; ++i) {
        b[i] = static_cast<char>((v >> (8 * i)) & 0xff);
    }
    o.write(b, 8);
}

void PutMap(std::string* o,
            const std::unordered_map<std::string, std::string>& kv) {
    PutU32(o, static_cast<uint32_t>(kv.size()));
    for (const auto& [k, v] : kv) {
        PutBytes(o, k);
        PutBytes(o, v);
    }
}

uint16_t ReadU16(const char* p) {
    return static_cast<uint16_t>(static_cast<unsigned char>(p[0])) |
           (static_cast<uint16_t>(static_cast<unsigned char>(p[1])) << 8);
}

uint32_t ReadU32(const char* p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
        v |= static_cast<uint32_t>(static_cast<unsigned char>(p[i])) << (8 * i);
    }
    return v;
}

uint64_t ReadU64(const char* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= static_cast<uint64_t>(static_cast<unsigned char>(p[i])) << (8 * i);
    }
    return v;
}

bool ReadString(const char* data, size_t size, size_t* off, std::string* out) {
    if (*off + 4 > size) {
        return false;
    }
    const uint32_t n = ReadU32(data + *off);
    *off += 4;
    if (*off + n > size) {
        return false;
    }
    out->assign(data + *off, n);
    *off += n;
    return true;
}

}  // namespace

McapWriter::~McapWriter() { Close(); }

bool McapWriter::Open(const std::string& path, const proto::EpisodeHeader& header) {
    Close();
    path_ = path;
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) {
        return false;
    }
    file_.write(kMagic, 8);
    WriteHeader();
    std::unordered_map<std::string, std::string> meta;
    meta["episode_id"] = header.episode_id();
    meta["policy_version"] = header.version().policy_version();
    meta["env_version"] = header.version().env_version();
    meta["git_sha"] = header.version().git_sha();
    meta["seed"] = header.version().seed();
    meta["admitted"] = header.admitted() ? "true" : "false";
    meta["time_source"] = proto::TimeSource_Name(header.time_source());
    meta["robot_id"] = header.robot_id();
    return WriteMetadata("autonomy.evidence", meta);
}

void McapWriter::WriteRecord(uint8_t opcode, const std::string& data) {
    WriteU8(file_, opcode);
    WriteU64(file_, data.size());
    WriteBytes(file_, data.data(), data.size());
}

std::string McapWriter::EncodeHeader() const {
    std::string d;
    PutBytes(&d, "autonomy");
    PutBytes(&d, "autonomy.recorder cpp");
    return d;
}

void McapWriter::WriteHeader() { WriteRecord(kHeader, EncodeHeader()); }

std::string McapWriter::EncodeSchema(uint16_t id, const std::string& name,
                                     const std::string& data) const {
    std::string d;
    PutU16(&d, id);
    PutBytes(&d, name);
    PutBytes(&d, "protobuf");
    PutBytes(&d, data);
    return d;
}

std::string McapWriter::EncodeChannel(const ChannelInfo& ch) const {
    std::string d;
    PutU16(&d, ch.channel_id);
    PutU16(&d, ch.schema_id);
    PutBytes(&d, ch.topic);
    PutBytes(&d, "protobuf");
    PutMap(&d, {});
    return d;
}

std::string McapWriter::EncodeMetadata(
    const std::string& name,
    const std::unordered_map<std::string, std::string>& kv) const {
    std::string d;
    PutBytes(&d, name);
    PutMap(&d, kv);
    return d;
}

std::string McapWriter::EncodeStatistics() const {
    std::string d;
    PutU64(&d, message_count_);
    PutU16(&d, static_cast<uint16_t>(channels_written_.size()));
    PutU16(&d, static_cast<uint16_t>(channels_written_.size()));
    PutU32(&d, 0);
    PutU32(&d, metadata_count_);
    PutU32(&d, 0);
    PutU64(&d, message_start_);
    PutU64(&d, message_end_);
    PutU32(&d, static_cast<uint32_t>(channels_.size()));
    for (const auto& [topic, ch] : channels_) {
        PutU16(&d, ch.channel_id);
        PutU64(&d, ch.count);
        (void)topic;
    }
    return d;
}

std::string McapWriter::EncodeFooter(uint64_t summary_start,
                                     uint64_t summary_offset_start) const {
    std::string d;
    PutU64(&d, summary_start);
    PutU64(&d, summary_offset_start);
    PutU32(&d, 0);
    return d;
}

uint16_t McapWriter::EnsureChannel(const Sample& sample) {
    auto it = channels_.find(sample.channel);
    if (it != channels_.end()) {
        return it->second.channel_id;
    }
    ChannelInfo ch;
    ch.schema_id = next_schema_++;
    ch.channel_id = next_channel_++;
    ch.topic = sample.channel;
    ch.schema_name =
        sample.message_type.empty() ? "automsgs.bytes" : sample.message_type;
    ch.schema_data = sample.proto_desc;
    WriteRecord(kSchema,
                EncodeSchema(ch.schema_id, ch.schema_name, ch.schema_data));
    WriteRecord(kChannel, EncodeChannel(ch));
    schemas_written_.emplace_back(ch.schema_name, ch.schema_data);
    channels_written_.push_back(ch);
    channels_.emplace(sample.channel, ch);
    return ch.channel_id;
}

bool McapWriter::WriteSample(const Sample& sample) {
    if (!file_.is_open()) {
        return false;
    }
    const uint16_t id = EnsureChannel(sample);
    const uint64_t t = static_cast<uint64_t>(sample.clock.monotonic_ns());
    const uint64_t body = 22ull + sample.payload.size();
    WriteU8(file_, kMessage);
    WriteU64(file_, body);
    WriteU16(file_, id);
    WriteU32(file_, sample.sequence);
    WriteU64(file_, t);
    WriteU64(file_, static_cast<uint64_t>(sample.clock.wall_utc_ns()));
    WriteBytes(file_, sample.payload.data(), sample.payload.size());
    auto it = channels_.find(sample.channel);
    if (it != channels_.end()) {
        ++it->second.count;
    }
    ++message_count_;
    if (message_count_ == 1) {
        message_start_ = t;
    }
    message_end_ = t;
    return static_cast<bool>(file_);
}

bool McapWriter::WriteMetadata(
    const std::string& name,
    const std::unordered_map<std::string, std::string>& kv) {
    if (!file_.is_open()) {
        return false;
    }
    const std::string rec = EncodeMetadata(name, kv);
    WriteRecord(kMetadata, rec);
    metadata_records_.push_back(rec);
    ++metadata_count_;
    return static_cast<bool>(file_);
}

bool McapWriter::Close() {
    if (!file_.is_open()) {
        return true;
    }
    std::string data_end;
    PutU32(&data_end, 0);
    WriteRecord(kDataEnd, data_end);

    const uint64_t summary_start =
        static_cast<uint64_t>(file_.tellp());
    for (size_t i = 0; i < channels_written_.size(); ++i) {
        const auto& ch = channels_written_[i];
        WriteRecord(kSchema,
                    EncodeSchema(ch.schema_id, ch.schema_name, ch.schema_data));
        WriteRecord(kChannel, EncodeChannel(ch));
    }
    for (const auto& rec : metadata_records_) {
        WriteRecord(kMetadata, rec);
    }
    WriteRecord(kStatistics, EncodeStatistics());
    const uint64_t summary_offset_start =
        static_cast<uint64_t>(file_.tellp());
    WriteRecord(kFooter, EncodeFooter(summary_start, summary_offset_start));
    file_.write(kMagic, 8);
    file_.flush();
    file_.close();
    channels_.clear();
    channels_written_.clear();
    schemas_written_.clear();
    metadata_records_.clear();
    message_count_ = 0;
    metadata_count_ = 0;
    next_schema_ = 1;
    next_channel_ = 1;
    return true;
}

std::vector<McapMessage> RecoverMcap(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::vector<McapMessage> out;
    if (!in) {
        return out;
    }
    std::array<char, 8> magic{};
    in.read(magic.data(), 8);
    if (in.gcount() != 8 || std::memcmp(magic.data(), kMagic, 8) != 0) {
        return out;
    }

    struct Ch {
        std::string topic;
        std::string schema;
    };
    std::unordered_map<uint16_t, Ch> channels;
    std::unordered_map<uint16_t, std::string> schemas;

    while (in) {
        uint8_t opcode = 0;
        in.read(reinterpret_cast<char*>(&opcode), 1);
        if (in.gcount() != 1) {
            break;
        }
        std::array<char, 8> lenb{};
        in.read(lenb.data(), 8);
        if (in.gcount() != 8) {
            break;
        }
        const uint64_t len = ReadU64(lenb.data());
        std::string data(len, '\0');
        in.read(data.data(), static_cast<std::streamsize>(len));
        if (static_cast<uint64_t>(in.gcount()) != len) {
            break;
        }
        if (opcode == kSchema) {
            if (data.size() < 2) {
                continue;
            }
            const uint16_t id = ReadU16(data.data());
            size_t off = 2;
            std::string name;
            std::string enc;
            std::string sdata;
            if (!ReadString(data.data(), data.size(), &off, &name) ||
                !ReadString(data.data(), data.size(), &off, &enc) ||
                !ReadString(data.data(), data.size(), &off, &sdata)) {
                continue;
            }
            schemas[id] = name;
        } else if (opcode == kChannel) {
            if (data.size() < 4) {
                continue;
            }
            const uint16_t cid = ReadU16(data.data());
            const uint16_t sid = ReadU16(data.data() + 2);
            size_t off = 4;
            std::string topic;
            std::string enc;
            if (!ReadString(data.data(), data.size(), &off, &topic) ||
                !ReadString(data.data(), data.size(), &off, &enc)) {
                continue;
            }
            channels[cid] = Ch{topic, schemas[sid]};
        } else if (opcode == kMessage) {
            if (data.size() < 2 + 4 + 8 + 8) {
                continue;
            }
            const uint16_t cid = ReadU16(data.data());
            McapMessage msg;
            msg.sequence = ReadU32(data.data() + 2);
            msg.log_time_ns = ReadU64(data.data() + 6);
            msg.publish_time_ns = ReadU64(data.data() + 14);
            msg.payload.assign(data.data() + 22, data.size() - 22);
            auto it = channels.find(cid);
            if (it != channels.end()) {
                msg.topic = it->second.topic;
                msg.schema_name = it->second.schema;
            }
            msg.encoding = "protobuf";
            out.push_back(std::move(msg));
        } else if (opcode == kDataEnd || opcode == kFooter) {
            break;
        }
    }
    return out;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
