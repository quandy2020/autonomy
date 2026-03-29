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

#include "autolink/sysmo/sysmo.hpp"

#include <cstdlib>
#include <string>

namespace autolink {

SysMo::SysMo() {
    Start();
}

void SysMo::Start() {
    // Optional env: do not use common::GetEnv (it warns when unset).
    const char* raw = std::getenv("sysmo_start");
    if (raw == nullptr || raw[0] == '\0') {
        return;
    }
    int flag = 0;
    try {
        flag = std::stoi(std::string(raw));
    } catch (...) {
        return;
    }
    if (flag) {
        start_ = true;
        sysmo_ = std::thread(&SysMo::Checker, this);
    }
}

void SysMo::Shutdown() {
    if (!start_ || shut_down_.exchange(true)) {
        return;
    }

    cv_.notify_all();
    if (sysmo_.joinable()) {
        sysmo_.join();
    }
}

void SysMo::Checker() {
    while (autolink_unlikely(!shut_down_.load())) {
        scheduler::Instance()->CheckSchedStatus();
        std::unique_lock<std::mutex> lk(lk_);
        cv_.wait_for(lk, std::chrono::milliseconds(sysmo_interval_ms_));
    }
}

}  // namespace autolink
