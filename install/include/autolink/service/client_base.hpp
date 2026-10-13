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

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

#include "autolink/common/macros.hpp"
#include "autolink/common/types.hpp"
#include "autolink/service_discovery/topology_manager.hpp"

namespace autolink {

/**
 * @class ClientBase
 * @brief Base class of Client
 *
 */
class ClientBase
{
public:
    /**
     * @brief Construct a new Client Base object
     *
     * @param service_name the service we can request
     */
    explicit ClientBase(const std::string& service_name)
        : service_name_(service_name) {}
    virtual ~ClientBase() {}

    /**
     * @brief Destroy the Client
     */
    virtual void Destroy() = 0;

    /**
     * @brief Get the service name
     */
    const std::string& ServiceName() const {
        return service_name_;
    }

    /**
     * @brief Ensure whether there is any Service named `service_name_`
     */
    virtual bool ServiceIsReady() const = 0;

protected:
    std::string service_name_;

    bool WaitForServiceNanoseconds(std::chrono::nanoseconds time_out) {
        auto* topology = service_discovery::TopologyManager::Instance();
        const std::string request_channel =
            service_name_ + SRV_CHANNEL_REQ_SUFFIX;
        const auto ready = [&]() {
            const bool has_reader =
                topology->channel_manager()->HasReader(request_channel);
            if (!has_reader) {
                return false;
            }
            if (topology->service_manager()->HasService(service_name_)) {
                return true;
            }
            // ROLE_SERVER can lag behind __SRV__ channels; accept writer peer.
            const std::string response_channel =
                service_name_ + SRV_CHANNEL_RES_SUFFIX;
            return topology->channel_manager()->HasWriter(response_channel);
        };
        if (ready()) {
            return true;
        }

        std::mutex mu;
        std::condition_variable cv;
        auto conn = topology->AddChangeListener(
            [&](const service_discovery::ChangeMsg& /*msg*/) {
                cv.notify_all();
            });
        struct ListenerGuard {
            service_discovery::TopologyManager* topology;
            service_discovery::TopologyManager::ChangeConnection conn;
            ~ListenerGuard() {
                topology->RemoveChangeListener(conn);
            }
        } guard{topology, conn};

        const bool wait_forever = time_out.count() < 0;
        const auto deadline =
            std::chrono::steady_clock::now() +
            (wait_forever ? std::chrono::hours(24 * 365) : time_out);
        std::unique_lock<std::mutex> lock(mu);
        while (wait_forever || std::chrono::steady_clock::now() < deadline) {
            if (ready()) {
                return true;
            }
            if (wait_forever) {
                cv.wait_for(lock, std::chrono::milliseconds(50));
            } else {
                cv.wait_until(lock, deadline);
            }
        }
        return ready();
    }
};

}  // namespace autolink
