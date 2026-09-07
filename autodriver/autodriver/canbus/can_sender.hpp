/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Periodic CAN frame sender (simplified).
 */

#ifndef AUTODRIVER_CANBUS_CAN_SENDER_HPP_
#define AUTODRIVER_CANBUS_CAN_SENDER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

#include "autodriver/canbus/can_client.hpp"
#include "autodriver/common/can_socket.hpp"

namespace autodriver {
namespace canbus {

/**
 * @class autodriver::canbus::CanSender
 * @brief Sends frames on a fixed interval via an owned CanClient pointer.
 *
 * Typical use: radar configuration frames or periodic keep-alive. The client
 * must outlive the sender; AddJob before Start().
 */
class CanSender {
public:
    // Builds one classical CAN frame for a scheduled job.
    using FrameBuilder = std::function<io::CanFrame()>;

    /**
     * @brief One periodic TX job.
     */
    struct Job {
        // Invoked each period to produce the frame to send.
        FrameBuilder build;
        // Send period in milliseconds (clamped to ≥1 when scheduled).
        int period_ms = 20;
    };

    /**
     * @brief Construct a sender bound to @p client (non-owning).
     * @param client Open CanClient used for Send(); must outlive this object.
     */
    explicit CanSender(CanClient* client) : client_(client) {}

    /**
     * @brief Stops the worker thread on destruction.
     */
    ~CanSender() { Stop(); }

    /**
     * @brief Register a periodic frame builder.
     * @param build Callback that returns the next frame.
     * @param period_ms Interval between sends; values ≤0 become 20 ms.
     */
    void AddJob(FrameBuilder build, int period_ms) {
        jobs_.push_back(Job{std::move(build), period_ms > 0 ? period_ms : 20});
    }

    /**
     * @brief Start the background send loop.
     * @return False when client is null; true if already running or started.
     */
    bool Start() {
        if (!client_ || running_.exchange(true)) {
            return client_ != nullptr && running_.load();
        }
        worker_ = std::thread([this]() { Loop(); });
        return true;
    }

    /**
     * @brief Stop the loop and join the worker.
     */
    void Stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    /**
     * @brief Whether the send loop is active.
     */
    bool IsRunning() const { return running_.load(); }

private:
    /**
     * @brief Worker: for each due job, build a frame and client_->Send.
     */
    void Loop() {
        using clock = std::chrono::steady_clock;
        std::vector<clock::time_point> next(jobs_.size(), clock::now());
        while (running_.load()) {
            const auto now = clock::now();
            for (std::size_t i = 0; i < jobs_.size(); ++i) {
                if (now < next[i]) {
                    continue;
                }
                if (jobs_[i].build && client_) {
                    client_->Send(jobs_[i].build());
                }
                next[i] = now + std::chrono::milliseconds(jobs_[i].period_ms);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Non-owning CAN backend.
    CanClient* client_ = nullptr;

    // Scheduled TX jobs.
    std::vector<Job> jobs_;

    // True while Loop() should run.
    std::atomic<bool> running_{false};
    
    // Background sender thread.
    std::thread worker_;
};

}  // namespace canbus
}  // namespace autodriver

#endif  // AUTODRIVER_CANBUS_CAN_SENDER_HPP_
