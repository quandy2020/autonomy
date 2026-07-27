/*
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

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

namespace autoviz {
namespace transform {
namespace tf2 {

/** Lightweight replacement for boost::signals2::signal<void()>. */
class VoidSignal
{
public:
    class Connection
    {
    public:
        Connection() = default;

        Connection(const Connection&) = default;
        Connection& operator=(const Connection&) = default;

        void disconnect() {
            if (signal_ != nullptr && slot_id_ != 0) {
                signal_->disconnect(slot_id_);
                slot_id_ = 0;
            }
        }

    private:
        friend class VoidSignal;
        Connection(VoidSignal* signal, uint64_t slot_id)
            : signal_(signal), slot_id_(slot_id) {}

        VoidSignal* signal_{nullptr};
        uint64_t slot_id_{0};
    };

    Connection connect(std::function<void()> callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t id = ++next_id_;
        slots_.push_back(Slot{id, std::move(callback), true});
        return Connection(this, id);
    }

    void disconnect(uint64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& slot : slots_) {
            if (slot.id == id) {
                slot.connected = false;
                return;
            }
        }
    }

    void operator()() {
        std::vector<std::function<void()>> callbacks;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& slot : slots_) {
                if (slot.connected && slot.callback) {
                    callbacks.push_back(slot.callback);
                }
            }
        }
        for (auto& cb : callbacks) {
            cb();
        }
    }

private:
    struct Slot {
        uint64_t id;
        std::function<void()> callback;
        bool connected;
    };

    std::mutex mutex_;
    std::vector<Slot> slots_;
    uint64_t next_id_{0};
};

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz
