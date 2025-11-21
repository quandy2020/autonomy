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

#include <netinet/in.h>
#include <atomic>

#include "autolink/common/macros.hpp"
#include "autolink/transport/shm/notifier_base.hpp"

namespace autolink {
namespace transport {

class MulticastNotifier : public NotifierBase
{
public:
    virtual ~MulticastNotifier();

    void Shutdown() override;
    bool Notify(const ReadableInfo& info) override;
    bool Listen(int timeout_ms, ReadableInfo* info) override;

    static const char* Type() {
        return "multicast";
    }

private:
    bool Init();

    int notify_fd_ = -1;
    struct sockaddr_in notify_addr_;
    int listen_fd_ = -1;
    struct sockaddr_in listen_addr_;

    std::atomic<bool> is_shutdown_ = {false};

    DECLARE_SINGLETON(MulticastNotifier)
};

}  // namespace transport
}  // namespace autolink