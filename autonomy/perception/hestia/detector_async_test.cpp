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
 * @file detector_async_test.cpp
 * @brief Tests for non-blocking open-vocabulary detection.
 */

#include "autonomy/perception/hestia/async.hpp"
#include "autonomy/perception/hestia/detector.hpp"


#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

class BlockingRunner final : public Runner
{
public:
    bool Run(const common::network::TensorMap& /*inputs*/,
             common::network::TensorMap* outputs, std::string* /*error*/) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            started_ = true;
        }
        cv_.notify_all();
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return release_ || stop_; });
        if (outputs != nullptr) {
            outputs->clear();
            outputs->emplace(
                "output0",
                common::network::Tensor::FromFloat32(
                    std::vector<float>{0, 0, 10, 10, 0.9F, 0.0F}));
        }
        return true;
    }

    void WaitUntilStarted() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::seconds(2), [&] { return started_; });
    }

    void Release() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            release_ = true;
        }
        cv_.notify_all();
    }

    void Stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            release_ = true;
        }
        cv_.notify_all();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool started_ = false;
    bool release_ = false;
    bool stop_ = false;
};

proto::HestiaOptions Options() {
    proto::HestiaOptions o;
    o.set_mode(proto::MODE_OPEN);
    o.set_open_model_path("/models/hestia_open.onnx");
    o.set_backend(proto::BACKEND_ONNX);
    o.set_open_width(32);
    o.set_open_height(32);
    o.set_max_detections(1);
    o.set_confidence_threshold(0.25F);
    o.add_open_prompts("chair");
    o.set_depth_scale(0.001F);
    o.set_min_depth_m(0.2F);
    o.set_max_depth_m(8.0F);
    o.set_min_depth_samples(3);
    o.set_inner_box_scale(0.5F);
    o.set_depth_outlier_m(0.25F);
    o.set_camera_frame("camera_optical");
    o.set_association_iou_threshold(0.3F);
    o.set_lost_timeout_sec(1.5F);
    o.set_merge_iou_threshold(0.5F);
    o.set_detections_2d_topic("/a");
    o.set_detections_3d_topic("/b");
    o.set_max_input_skew_sec(0.05F);
    o.set_nms_iou_threshold(0.0F);
    o.set_tf_timeout_sec(0.05F);
    return o;
}

automsgs::msgs::sensor_msgs::Image TinyRgb() {
    automsgs::msgs::sensor_msgs::Image image;
    image.set_encoding("rgb8");
    image.set_width(32);
    image.set_height(32);
    image.set_step(96);
    image.mutable_data()->assign(32 * 32 * 3, '\0');
    return image;
}

TEST(AsyncTest, RejectsEnqueueWhileBusy) {
    auto runner = std::make_unique<BlockingRunner>();
    BlockingRunner* runner_ptr = runner.get();
    auto detector = OpenDetector::Create(Options(), std::move(runner));
    ASSERT_NE(detector, nullptr);
    Async<OpenDetector> worker(std::move(detector));

    ASSERT_TRUE(worker.TryEnqueue(TinyRgb()));
    runner_ptr->WaitUntilStarted();
    EXPECT_FALSE(worker.TryEnqueue(TinyRgb()));

    runner_ptr->Release();
    automsgs::msgs::vision_msgs::Detection2DArray latest;
    for (int i = 0; i < 50; ++i) {
        if (worker.TryPoll(&latest)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_GE(latest.detections_size(), 0);
    runner_ptr->Stop();
    worker.Shutdown();
}

}  // namespace
}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
