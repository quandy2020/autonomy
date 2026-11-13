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

#pragma once

/**
 * @file mcap_writer.hpp
 * @brief Minimal MCAP writer: protobuf schema + direct payload write, no extra string pack.
 */

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"
#include "autonomy/system/recorder/rolling_buffer.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief One message recovered from an MCAP file (RecoverMcap).
 */
struct McapMessage {
    std::string topic;            ///< Topic name.
    std::string encoding;         ///< Encoding (protobuf).
    std::string schema_name;      ///< Schema name.
    std::string schema_data;      ///< FileDescriptorSet bytes.
    std::string payload;          ///< Message body.
    uint64_t log_time_ns{0};      ///< Log time.
    uint64_t publish_time_ns{0};  ///< Publish time.
    uint32_t sequence{0};         ///< Sequence number.
};

/**
 * @brief Single-file MCAP writer. Open writes Header; Close writes Summary + Footer.
 *
 * WriteSample writes opcode/len/header then payload directly, without assembling
 * a second full std::string.
 */
class McapWriter {
public:
    /**
     * @brief Close the file if still open.
     */
    ~McapWriter();

    /**
     * @brief Create the file and write MCAP Header plus episode metadata.
     * @param [in] path: Output path.
     * @param [in] header: EpisodeHeader stored as metadata.
     * @return True on success.
     */
    bool Open(const std::string& path, const proto::EpisodeHeader& header);

    /**
     * @brief Write one sample. Registers channel/schema on first appearance.
     * @param [in] sample: Sample with payload.
     * @return True on success.
     */
    bool WriteSample(const Sample& sample);

    /**
     * @brief Write a Metadata record (key/value).
     * @param [in] name: Metadata name.
     * @param [in] kv: Key/value map.
     * @return True on success.
     */
    bool WriteMetadata(const std::string& name,
                       const std::unordered_map<std::string, std::string>& kv);

    /**
     * @brief Write Summary / Footer and close the file.
     * @return True on success.
     */
    bool Close();

    /**
     * @brief Whether the file is open.
     * @return True if open.
     */
    bool is_open() const { return file_.is_open(); }

    /**
     * @brief Current file path.
     * @return Path; empty before Open.
     */
    const std::string& path() const { return path_; }

private:
    /**
     * @brief Registered MCAP channel.
     */
    struct ChannelInfo {
        uint16_t channel_id{0};   ///< MCAP channel id.
        uint16_t schema_id{0};    ///< MCAP schema id.
        std::string topic;        ///< Topic name.
        std::string schema_name;  ///< Schema name.
        std::string schema_data;  ///< Descriptor bytes.
        uint64_t count{0};        ///< Messages on this channel.
    };

    /**
     * @brief Write opcode + length + data.
     * @param [in] opcode: MCAP opcode.
     * @param [in] data: Record body.
     */
    void WriteRecord(uint8_t opcode, const std::string& data);

    /**
     * @brief Write the MCAP Header record.
     */
    void WriteHeader();

    /**
     * @brief Register the channel; write Schema/Channel records if new.
     * @param [in] sample: Sample.
     * @return Channel id.
     */
    uint16_t EnsureChannel(const Sample& sample);

    /**
     * @brief Encode a Header record body.
     * @return Bytes.
     */
    std::string EncodeHeader() const;

    /**
     * @brief Encode a Schema record body.
     * @param [in] id: Schema id.
     * @param [in] name: Type name.
     * @param [in] data: FileDescriptorSet.
     * @return Bytes.
     */
    std::string EncodeSchema(uint16_t id, const std::string& name,
                             const std::string& data) const;

    /**
     * @brief Encode a Channel record body.
     * @param [in] ch: Channel info.
     * @return Bytes.
     */
    std::string EncodeChannel(const ChannelInfo& ch) const;

    /**
     * @brief Encode a Metadata record body.
     * @param [in] name: Name.
     * @param [in] kv: Key/value map.
     * @return Bytes.
     */
    std::string EncodeMetadata(
        const std::string& name,
        const std::unordered_map<std::string, std::string>& kv) const;

    /**
     * @brief Encode a Statistics record body.
     * @return Bytes.
     */
    std::string EncodeStatistics() const;

    /**
     * @brief Encode a Footer record.
     * @param [in] summary_start: Summary start offset.
     * @param [in] summary_offset_start: SummaryOffset start.
     * @return Bytes.
     */
    std::string EncodeFooter(uint64_t summary_start,
                             uint64_t summary_offset_start) const;

    std::string path_;
    std::ofstream file_;
    uint16_t next_schema_{1};
    uint16_t next_channel_{1};
    std::unordered_map<std::string, ChannelInfo> channels_;
    uint64_t message_count_{0};
    uint64_t message_start_{0};
    uint64_t message_end_{0};
    uint32_t metadata_count_{0};
    std::vector<std::pair<std::string, std::string>> schemas_written_;
    std::vector<ChannelInfo> channels_written_;
    std::vector<std::string> metadata_records_;
};

/**
 * @brief Scan an MCAP and recover messages (partial read of a truncated file is OK).
 * @param [in] path: MCAP path.
 * @return Recovered messages.
 */
std::vector<McapMessage> RecoverMcap(const std::string& path);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
