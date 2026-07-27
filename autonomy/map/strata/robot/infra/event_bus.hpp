/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace infra {

class EventBus
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(EventBus)

    using Handler = std::function<void(const std::string&, const std::string&)>;

    void On(const std::string& signal, Handler handler);
    void Once(const std::string& signal, Handler handler);
    void Off(const std::string& signal);
    void Emit(const std::string& signal, const std::string& payload = {});
    const std::unordered_map<std::string, std::string>& ActiveSignals() const;
    std::string GetSignalPayload(const std::string& signal) const;
    std::future<void> WaitFor(const std::string& signal);
    void Clear();

private:
    struct HandlerEntry {
        Handler handler;
        bool once{false};
    };

    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::vector<HandlerEntry>> handlers_;
    std::unordered_map<std::string, std::vector<std::promise<void>>> waiters_;
    std::unordered_map<std::string, std::string> active_signals_;
};

enum class ResourceType { kElevator, kDoor, kChargingStation, kDock, kCustom };

enum class ResourceStatus { kFree, kBusy, kShared };

class ResourceManager
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ResourceManager)

    void RegisterResource(ResourceType resource_type, const std::string& resource_id,
                          bool shared = false);
    bool Acquire(const std::string& resourceId, const std::string& robotId,
                 bool shared = false);
    void Release(const std::string& resourceId, const std::string& robotId);
    bool IsLocked(const std::string& resourceId) const;
    std::string GetOwner(const std::string& resourceId) const;
    ResourceStatus GetStatus(ResourceType resource_type,
                             const std::string& resource_id) const;

private:
    struct ResourceEntry {
        ResourceType type{ResourceType::kCustom};
        ResourceStatus status{ResourceStatus::kFree};
        bool shared{false};
        std::string owner;
    };

    static std::string MakeKey(ResourceType resource_type, const std::string& resource_id);

    mutable std::mutex mutex_;
    std::unordered_map<std::string, ResourceEntry> resources_;
    std::unordered_map<std::string, std::string> locks_;
};

}  // namespace infra
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
