/*
 * Copyright 2026 The Openbot Authors
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

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core/mat.hpp>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/viz_bridge.hpp"
#include "autonomy/localization/atlas/map/dense_map_builder.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * Autolink Image → OpenVSLAM feed_*_frame bridge.
 *
 * Monocular: /rgb → feed_monocular_frame
 * RGBD:      /rgb + /depth (exact stamp match preferred) → feed_RGBD_frame
 */
class ImageBridge {
public:
    struct Options {
        std::string rgb_topic = "/camera/rgb/image_raw";
        std::string depth_topic = "/camera/depth/image_raw";
        /** Max |rgb_stamp - depth_stamp| for RGBD pairing (seconds). */
        double sync_slop_sec = 0.05;
        /** How many recent frames to keep per stream while waiting for a pair. */
        std::size_t sync_queue_size = 8;
    };

    ImageBridge(system* slam, Options options);
    ~ImageBridge();

    ImageBridge(const ImageBridge&) = delete;
    ImageBridge& operator=(const ImageBridge&) = delete;

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

    /** Optional frontend visualization (features / matches / path / FOV). */
    void SetVizBridge(VizBridge* viz) { viz_ = viz; }
    /** Optional dense cloud + occupancy grid builder (RGB-D). */
    void SetDenseMapBuilder(map::DenseMapBuilder* dense) { dense_map_ = dense; }

private:
    struct BufferedFrame {
        cv::Mat image;
        int64_t stamp_ns = -1;
        double timestamp_sec = 0.0;
        bool valid() const { return stamp_ns >= 0 && !image.empty(); }
    };

    void OnRgb(const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg);
    void OnDepth(const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg);

    void PushFrame(std::deque<BufferedFrame>* queue, BufferedFrame frame);
    /** Prefer exact stamp_ns equality; else closest pair within sync_slop. */
    bool TakeSyncedRgbd(cv::Mat* rgb, cv::Mat* depth, double* timestamp_sec);

    void FeedMonocular(const cv::Mat& rgb, double timestamp_sec);
    void FeedRgbd(const cv::Mat& rgb, const cv::Mat& depth, double timestamp_sec);

    system* slam_ = nullptr;
    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::atomic<bool> running_{false};
    bool is_rgbd_ = false;
    VizBridge* viz_ = nullptr;
    map::DenseMapBuilder* dense_map_ = nullptr;

    std::mutex mutex_;
    std::deque<BufferedFrame> pending_rgb_;
    std::deque<BufferedFrame> pending_depth_;
    uint64_t rgb_count_ = 0;
    uint64_t depth_count_ = 0;
    uint64_t fed_count_ = 0;
    uint64_t drop_count_ = 0;
};

/** Convert sensor_msgs/Image to cv::Mat (copies data). */
bool ImageMsgToCvMat(const automsgs::msgs::sensor_msgs::Image& msg,
                     cv::Mat* out);

/** Header stamp → seconds (double) for OpenVSLAM timestamps. */
double StampToSeconds(const automsgs::msgs::sensor_msgs::Image& msg);

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
