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
 * @file open_worker.hpp
 * @brief Non-blocking open-vocabulary inference worker for dual mode.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_OPEN_WORKER_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_OPEN_WORKER_HPP_

#include "autonomy/perception/hestia/detector.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace autonomy {
namespace perception {
namespace hestia {

/**
 * @brief Runs OpenDetector on a background thread without blocking Proc.
 *
 * TrySubmit returns false when a job is already in flight (queue depth 1).
 * queue_depth is accepted for API compatibility; v1 uses a single slot.
 */
class OpenAsyncWorker
{
public:
    OpenAsyncWorker(std::unique_ptr<OpenDetector> detector,
                    uint32_t queue_depth);
    ~OpenAsyncWorker();

    OpenAsyncWorker(const OpenAsyncWorker&) = delete;
    OpenAsyncWorker& operator=(const OpenAsyncWorker&) = delete;

    bool TrySubmit(const automsgs::msgs::sensor_msgs::Image& rgb);
    bool TryGetLatest(
        automsgs::msgs::vision_msgs::Detection2DArray* out) const;
    void Shutdown();

private:
    void Loop();

    std::unique_ptr<OpenDetector> detector_;
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

#endif  // AUTONOMY_PERCEPTION_HESTIA_OPEN_WORKER_HPP_
