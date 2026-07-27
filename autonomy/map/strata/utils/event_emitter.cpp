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

#include "autonomy/map/strata/utils/event_emitter.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace utils {

void EventEmitter::On(const std::string& event, Handler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    listeners_[event].push_back({std::move(handler), false});
}

void EventEmitter::Once(const std::string& event, Handler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    listeners_[event].push_back({std::move(handler), true});
}

void EventEmitter::Off(const std::string& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    listeners_.erase(event);
}

void EventEmitter::Emit(const std::string& event, const std::string& payload) {
    std::vector<Listener> snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = listeners_.find(event);
        if (it == listeners_.end()) {
            return;
        }
        snapshot = it->second;
    }

    std::vector<Listener> remaining;
    for (auto& listener : snapshot) {
        listener.handler(event, payload);
        if (!listener.once) {
            remaining.push_back(std::move(listener));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    listeners_[event] = std::move(remaining);
}

}  // namespace utils
}  // namespace strata
}  // namespace map
}  // namespace autonomy
