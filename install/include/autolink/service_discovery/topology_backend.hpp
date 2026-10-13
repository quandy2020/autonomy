/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <cstdint>
#include <atomic>
#include <chrono>
#include <functional>
#include <ios>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autolink/proto/topology_change.pb.h"

namespace autolink {
namespace service_discovery {

class ITopologyBackend
{
public:
    using ChangeCallback = std::function<void(const proto::ChangeMsg&)>;

    virtual ~ITopologyBackend() = default;

    virtual bool Start() = 0;
    virtual int64_t Subscribe(proto::ChangeType type, const ChangeCallback& callback) = 0;
    virtual void Unsubscribe(int64_t subscription_id) = 0;
    virtual bool Publish(const proto::ChangeMsg& msg) = 0;
    virtual void Shutdown() = 0;
};

class LocalTopologyBackend final : public ITopologyBackend
{
public:
    bool Start() override;
    int64_t Subscribe(proto::ChangeType type, const ChangeCallback& callback) override;
    void Unsubscribe(int64_t subscription_id) override;
    bool Publish(const proto::ChangeMsg& msg) override;
    void Shutdown() override;

private:
    struct ProcessState {
        std::chrono::steady_clock::time_point last_seen;
        std::unordered_map<std::string, proto::ChangeMsg> active_roles;
    };

    void PollLoop();
    void DispatchMessage(const proto::ChangeMsg& msg);
    void HandleIncomingMessage(const proto::ChangeMsg& msg);
    void PublishHeartbeat();
    void CleanupStaleProcesses();
    void MaybeCompactEventFile();
    std::string ProcessKey(const proto::RoleAttributes& attr) const;
    std::string RoleKey(const proto::ChangeMsg& msg) const;

    struct Subscriber {
        proto::ChangeType type;
        ChangeCallback callback;
    };

    std::atomic<bool> running_ = {false};
    std::thread poll_thread_;
    int64_t next_subscription_id_ = 1;
    std::mutex subscribers_mutex_;
    std::unordered_map<int64_t, Subscriber> subscribers_;
    std::mutex publish_mutex_;
    std::string event_file_path_ = "/tmp/autolink_topology_events.log";
    std::streamoff read_offset_ = 0;
    std::chrono::steady_clock::time_point next_heartbeat_at_;
    std::chrono::steady_clock::time_point next_cleanup_at_;
    std::chrono::steady_clock::time_point next_compact_at_;
    std::string self_host_name_;
    int self_process_id_ = 0;
    std::string self_process_key_;
    std::mutex state_mutex_;
    std::unordered_map<std::string, ProcessState> process_states_;
};

}  // namespace service_discovery
}  // namespace autolink
