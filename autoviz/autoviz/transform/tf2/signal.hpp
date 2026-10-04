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

/**
 * @file signal.hpp
 * @brief Lightweight @c void() signal/slot replacement for boost::signals2.
 *
 * Used by Autoviz tf2 (@c BufferCore transforms-changed notifications) without
 * depending on Boost.Signals2. Slots are invoked synchronously from
 * @ref VoidSignal::operator()().
 *
 * @see tf2::BufferCore
 */

#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @class VoidSignal
 * @brief Thread-safe multicast @c void() callback list.
 *
 * ## Usage
 *
 * @code
 *   VoidSignal signal;
 *   auto conn = signal.connect([]{ ... });
 *   signal();           // invoke all connected slots
 *   conn.disconnect();  // or signal.disconnect(id)
 * @endcode
 *
 * @note @c operator()() copies connected callbacks under the lock, then
 *       invokes them outside the lock to avoid deadlocks if a slot
 *       connects/disconnects.
 */
class VoidSignal
{
public:
    /**
     * @class Connection
     * @brief RAII-ish handle that can disconnect a previously connected slot.
     *
     * Copyable; disconnecting one copy invalidates the slot for all copies
     * sharing the same @c slot_id_.
     */
    class Connection
    {
    public:
        /** @brief Default-constructs a disconnected / empty connection. */
        Connection() = default;

        Connection(const Connection&) = default;
        Connection& operator=(const Connection&) = default;

        /**
         * @brief Disconnects the associated slot if still connected.
         *
         * Safe to call multiple times; subsequent calls are no-ops.
         */
        void disconnect() {
            if (signal_ != nullptr && slot_id_ != 0) {
                signal_->disconnect(slot_id_);
                slot_id_ = 0;
            }
        }

    private:
        friend class VoidSignal;

        /**
         * @brief Constructs a connection for @p slot_id on @p signal.
         *
         * @param signal Owning signal (non-owning pointer).
         * @param slot_id Slot id assigned by @ref VoidSignal::connect.
         */
        Connection(VoidSignal* signal, uint64_t slot_id)
            : signal_(signal), slot_id_(slot_id) {}

        /** Non-owning pointer to the parent signal. */
        VoidSignal* signal_{nullptr};

        /** Slot id; @c 0 means disconnected / empty. */
        uint64_t slot_id_{0};
    };

    /**
     * @brief Registers @p callback and returns a disconnect handle.
     *
     * @param callback Slot invoked by @ref operator()().
     * @return @ref Connection for later @c disconnect().
     */
    Connection connect(std::function<void()> callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t id = ++next_id_;
        slots_.push_back(Slot{id, std::move(callback), true});
        return Connection(this, id);
    }

    /**
     * @brief Marks the slot with @p id as disconnected.
     *
     * @param id Slot id from @ref connect / @ref Connection.
     *
     * @note Does not erase the slot entry immediately; disconnected slots are
     *       skipped during emit (lazy tombstone).
     */
    void disconnect(uint64_t id) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& slot : slots_) {
            if (slot.id == id) {
                slot.connected = false;
                return;
            }
        }
    }

    /**
     * @brief Invokes all currently connected slots (synchronously).
     *
     * Copies callbacks under @c mutex_, then calls them unlocked.
     */
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
    /**
     * @struct Slot
     * @brief One registered callback entry.
     */
    struct Slot {
        /** Unique id within this signal. */
        uint64_t id;
        /** User callback. */
        std::function<void()> callback;
        /** @c false after @ref disconnect. */
        bool connected;
    };

    /** Guards @c slots_ and @c next_id_. */
    std::mutex mutex_;

    /** Registered slots (including disconnected tombstones). */
    std::vector<Slot> slots_;

    /** Monotonic id allocator for new slots. */
    uint64_t next_id_{0};
};

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz
