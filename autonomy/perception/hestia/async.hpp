/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file async.hpp
 * @brief Single in-flight async detection slot.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_ASYNC_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_ASYNC_HPP_

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace autonomy {
namespace perception {
namespace hestia {

template <class DetectorT>
class Async
{
public:
    explicit Async(std::unique_ptr<DetectorT> detector)
        : detector_(std::move(detector)) {
        worker_ = std::thread([this] { Loop(); });
    }

    ~Async() { Shutdown(); }

    Async(const Async&) = delete;
    Async& operator=(const Async&) = delete;

    bool TryEnqueue(const automsgs::msgs::sensor_msgs::Image& rgb) {
        if (stop_ || detector_ == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || has_pending_) {
            return false;
        }
        pending_ = rgb;
        has_pending_ = true;
        busy_ = true;
        cv_.notify_one();
        return true;
    }

    bool TryPoll(automsgs::msgs::vision_msgs::Detection2DArray* out) const {
        if (out == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_latest_) {
            return false;
        }
        *out = latest_;
        return true;
    }

    void Shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            has_pending_ = false;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    void Loop() {
        while (true) {
            automsgs::msgs::sensor_msgs::Image job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] { return stop_ || has_pending_; });
                if (stop_ && !has_pending_) {
                    return;
                }
                job = std::move(pending_);
                has_pending_ = false;
            }

            automsgs::msgs::vision_msgs::Detection2DArray detections;
            std::string error;
            const bool ok = detector_ != nullptr &&
                            detector_->Detect(job, &detections, &error);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (ok) {
                    latest_ = std::move(detections);
                    has_latest_ = true;
                }
                busy_ = false;
            }
        }
    }

    std::unique_ptr<DetectorT> detector_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> busy_{false};
    bool has_pending_ = false;
    bool has_latest_ = false;
    automsgs::msgs::sensor_msgs::Image pending_;
    automsgs::msgs::vision_msgs::Detection2DArray latest_;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_ASYNC_HPP_
