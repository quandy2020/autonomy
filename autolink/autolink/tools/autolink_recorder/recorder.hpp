/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/proto/record.pb.h"
#include "autolink/proto/topology_change.pb.h"

#include "autolink/autolink.hpp"
#include "autolink/base/signal.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/record/record_writer.hpp"

using autolink::Node;
using autolink::ReaderBase;
using autolink::base::Connection;
using autolink::message::RawMessage;
using autolink::proto::ChangeMsg;
using autolink::proto::RoleAttributes;
using autolink::proto::RoleType;
using autolink::service_discovery::ChannelManager;
using autolink::service_discovery::TopologyManager;

namespace autolink {
namespace record {

class Recorder : public std::enable_shared_from_this<Recorder>
{
public:
    Recorder(const std::string& output, bool all_channels,
             const std::vector<std::string>& white_channels,
             const std::vector<std::string>& black_channels);
    Recorder(const std::string& output, bool all_channels,
             const std::vector<std::string>& white_channels,
             const std::vector<std::string>& black_channels,
             const proto::Header& header);
    ~Recorder();
    bool Start();
    bool Stop();

private:
    bool is_started_ = false;
    bool is_stopping_ = false;
    std::shared_ptr<Node> node_ = nullptr;
    std::shared_ptr<RecordWriter> writer_ = nullptr;
    std::shared_ptr<std::thread> display_thread_ = nullptr;
    Connection<const ChangeMsg&> change_conn_;
    std::string output_;
    bool all_channels_ = true;
    std::vector<std::string> white_channels_;
    std::vector<std::regex> white_channel_patterns_;
    std::vector<std::string> black_channels_;
    std::vector<std::regex> black_channel_patterns_;
    proto::Header header_;
    std::unordered_map<std::string, std::shared_ptr<ReaderBase>>
        channel_reader_map_;
    uint64_t message_count_;
    uint64_t message_time_;

    bool InitReadersImpl();

    bool FreeReadersImpl();

    bool InitReaderImpl(const std::string& channel_name,
                        const std::string& message_type);

    void TopologyCallback(const ChangeMsg& msg);

    void ReaderCallback(const std::shared_ptr<RawMessage>& message,
                        const std::string& channel_name);

    void FindNewChannel(const RoleAttributes& role_attr);

    void ShowProgress();
};

}  // namespace record
}  // namespace autolink