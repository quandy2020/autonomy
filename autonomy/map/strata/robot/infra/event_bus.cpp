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

#include "autonomy/map/strata/robot/infra/event_bus.hpp"

#include <algorithm>

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace infra {

void EventBus::On(const std::string& signal, Handler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_[signal].push_back(HandlerEntry{std::move(handler), false});
}

void EventBus::Once(const std::string& signal, Handler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_[signal].push_back(HandlerEntry{std::move(handler), true});
}

void EventBus::Off(const std::string& signal) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.erase(signal);
}

void EventBus::Emit(const std::string& signal, const std::string& payload) {
    std::vector<HandlerEntry> snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_signals_[signal] = payload;
        if (handlers_.count(signal)) {
            snapshot = handlers_[signal];
        }
        if (waiters_.count(signal)) {
            for (auto& promise : waiters_[signal]) {
                promise.set_value();
            }
            waiters_.erase(signal);
        }
    }
    for (auto& entry : snapshot) {
        entry.handler(signal, payload);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!handlers_.count(signal)) {
        return;
    }
    auto& entries = handlers_[signal];
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [](const HandlerEntry& entry) { return entry.once; }),
                  entries.end());
    if (entries.empty()) {
        handlers_.erase(signal);
    }
}

const std::unordered_map<std::string, std::string>& EventBus::ActiveSignals() const {
    return active_signals_;
}

std::string EventBus::GetSignalPayload(const std::string& signal) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = active_signals_.find(signal);
    return it == active_signals_.end() ? "" : it->second;
}

std::future<void> EventBus::WaitFor(const std::string& signal) {
    std::promise<void> promise;
    auto future = promise.get_future();
    std::lock_guard<std::mutex> lock(mutex_);
    waiters_[signal].push_back(std::move(promise));
    return future;
}

void EventBus::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.clear();
    waiters_.clear();
    active_signals_.clear();
}

std::string ResourceManager::MakeKey(ResourceType resource_type,
                                     const std::string& resource_id) {
    return std::to_string(static_cast<int>(resource_type)) + ":" + resource_id;
}

void ResourceManager::RegisterResource(ResourceType resource_type,
                                       const std::string& resource_id, bool shared) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key = MakeKey(resource_type, resource_id);
    auto& entry = resources_[key];
    entry.type = resource_type;
    entry.shared = shared;
    if (entry.status == ResourceStatus::kFree) {
        entry.status = ResourceStatus::kFree;
    }
}

bool ResourceManager::Acquire(const std::string& resourceId, const std::string& robotId,
                              bool shared) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = locks_.find(resourceId);
    if (it != locks_.end() && it->second != robotId) {
        return false;
    }
    locks_[resourceId] = robotId;
    for (auto& [key, entry] : resources_) {
        if (key.find(resourceId) != std::string::npos ||
            key.substr(key.find(':') + 1) == resourceId) {
            if (shared || entry.shared) {
                entry.status = ResourceStatus::kShared;
            } else {
                entry.status = ResourceStatus::kBusy;
            }
            entry.owner = robotId;
        }
    }
    return true;
}

void ResourceManager::Release(const std::string& resourceId, const std::string& robotId) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = locks_.find(resourceId);
    if (it != locks_.end() && it->second == robotId) {
        locks_.erase(it);
    }
    for (auto& [key, entry] : resources_) {
        if (entry.owner == robotId &&
            (key.find(resourceId) != std::string::npos ||
             key.substr(key.find(':') + 1) == resourceId)) {
            entry.status = ResourceStatus::kFree;
            entry.owner.clear();
        }
    }
}

bool ResourceManager::IsLocked(const std::string& resourceId) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return locks_.count(resourceId) > 0;
}

std::string ResourceManager::GetOwner(const std::string& resourceId) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = locks_.find(resourceId);
    return it == locks_.end() ? "" : it->second;
}

ResourceStatus ResourceManager::GetStatus(ResourceType resource_type,
                                          const std::string& resource_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key = MakeKey(resource_type, resource_id);
    const auto it = resources_.find(key);
    if (it != resources_.end()) {
        return it->second.status;
    }
    return locks_.count(resource_id) > 0 ? ResourceStatus::kBusy : ResourceStatus::kFree;
}

}  // namespace infra
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
