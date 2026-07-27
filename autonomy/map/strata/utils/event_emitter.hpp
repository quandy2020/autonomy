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
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace utils {

class EventEmitter
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(EventEmitter)

    using Handler = std::function<void(const std::string& event, const std::string& payload)>;

    void On(const std::string& event, Handler handler);
    void Once(const std::string& event, Handler handler);
    void Off(const std::string& event);
    void Emit(const std::string& event, const std::string& payload = {});

private:
    struct Listener {
        Handler handler;
        bool once{false};
    };

    std::mutex mutex_;
    std::unordered_map<std::string, std::vector<Listener>> listeners_;
};

}  // namespace utils
}  // namespace strata
}  // namespace map
}  // namespace autonomy
