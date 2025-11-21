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

#ifdef __linux__
#include <sys/epoll.h>
#elif defined(__APPLE__)
// epoll is Linux-specific, use kqueue on macOS
#include <sys/event.h>
#endif

#include <cstdint>
#include <functional>

namespace autolink {
namespace io {

struct PollResponse {
    explicit PollResponse(uint32_t e = 0) : events(e) {}

    uint32_t events;
};

struct PollRequest {
    int fd = -1;
    uint32_t events = 0;
    int timeout_ms = -1;
    std::function<void(const PollResponse&)> callback = nullptr;
};

struct PollCtrlParam {
    int operation;
    int fd;
#ifdef __linux__
    epoll_event event;
#elif defined(__APPLE__)
    struct kevent event;
#endif
};

}  // namespace io
}  // namespace autolink