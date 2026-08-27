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

#include "autonomy/localization/atlas/image_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "autonomy/common/logging.hpp"
#include "autonomy/localization/atlas/camera/base.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

int EncodingToCvType(const std::string& encoding) {
    if (encoding == "mono8") {
        return CV_8UC1;
    }
    if (encoding == "mono16" || encoding == "16UC1") {
        return CV_16UC1;
    }
    if (encoding == "bgr8" || encoding == "rgb8") {
        return CV_8UC3;
    }
    if (encoding == "bgra8" || encoding == "rgba8") {
        return CV_8UC4;
    }
    if (encoding == "32FC1") {
        return CV_32FC1;
    }
    return -1;
}

int64_t StampToNanoseconds(const automsgs::msgs::sensor_msgs::Image& msg) {
    const auto& stamp = msg.header().stamp();
    return static_cast<int64_t>(stamp.sec()) * 1000000000LL +
           static_cast<int64_t>(stamp.nanosec());
}

/** Depth as CV_32FC1 meters (16UC1 assumed millimeters). */
bool NormalizeDepth(cv::Mat* depth) {
    if (depth == nullptr || depth->empty()) {
        return false;
    }
    if (depth->type() == CV_32FC1) {
        return true;
    }
    if (depth->type() == CV_16UC1) {
        cv::Mat meters;
        depth->convertTo(meters, CV_32FC1, 1.0 / 1000.0);
        *depth = meters;
        return true;
    }
    AWARN << "Atlas ImageBridge: unsupported depth type " << depth->type();
    return false;
}

}  // namespace

bool ImageMsgToCvMat(const automsgs::msgs::sensor_msgs::Image& msg,
                     cv::Mat* out) {
    if (out == nullptr) {
        return false;
    }
    const int cv_type = EncodingToCvType(msg.encoding());
    if (cv_type < 0) {
        AWARN << "Atlas ImageBridge: unsupported encoding '" << msg.encoding()
              << "'";
        return false;
    }
    if (msg.width() == 0 || msg.height() == 0) {
        return false;
    }

    const size_t elem_size = CV_ELEM_SIZE(cv_type);
    const uint32_t step =
        msg.step() > 0
            ? msg.step()
            : static_cast<uint32_t>(msg.width()) *
                  static_cast<uint32_t>(elem_size);
    const size_t expected =
        static_cast<size_t>(msg.height()) * static_cast<size_t>(step);
    if (msg.data().size() < expected) {
        AWARN << "Atlas ImageBridge: image data too small (have "
              << msg.data().size() << ", need " << expected << ")";
        return false;
    }

    cv::Mat mat(static_cast<int>(msg.height()),
                static_cast<int>(msg.width()), cv_type);
    if (step == static_cast<uint32_t>(msg.width()) * elem_size) {
        std::memcpy(mat.data, msg.data().data(),
                    static_cast<size_t>(msg.height()) *
                        static_cast<size_t>(msg.width()) * elem_size);
    } else {
        for (uint32_t r = 0; r < msg.height(); ++r) {
            std::memcpy(mat.ptr(static_cast<int>(r)),
                        msg.data().data() +
                            static_cast<size_t>(r) * static_cast<size_t>(step),
                        static_cast<size_t>(msg.width()) * elem_size);
        }
    }
    *out = mat;
    return true;
}

double StampToSeconds(const automsgs::msgs::sensor_msgs::Image& msg) {
    const auto& stamp = msg.header().stamp();
    return static_cast<double>(stamp.sec()) +
           static_cast<double>(stamp.nanosec()) * 1e-9;
}

ImageBridge::ImageBridge(system* slam, Options options)
    : slam_(slam), options_(std::move(options)) {
    if (options_.sync_queue_size < 1) {
        options_.sync_queue_size = 1;
    }
    if (options_.sync_slop_sec < 0.0) {
        options_.sync_slop_sec = 0.0;
    }
}

ImageBridge::~ImageBridge() { Stop(); }

bool ImageBridge::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!slam_ || !node) {
        AERROR << "Atlas ImageBridge: missing system or autolink node.";
        return false;
    }
    if (running_) {
        return true;
    }

    camera::base* cam = slam_->get_camera();
    if (cam == nullptr) {
        AERROR << "Atlas ImageBridge: camera not ready.";
        return false;
    }

    node_ = node;
    running_ = true;
    is_rgbd_ = (cam->setup_type_ == camera::setup_type_t::RGBD);

    ImageBridge* self = this;
    node_->CreateReader<automsgs::msgs::sensor_msgs::Image>(
        options_.rgb_topic,
        [self](const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg) {
            if (msg) {
                self->OnRgb(msg);
            }
        });

    if (is_rgbd_) {
        if (options_.depth_topic.empty()) {
            AERROR << "Atlas ImageBridge: RGBD setup requires depth_topic.";
            running_ = false;
            return false;
        }
        node_->CreateReader<automsgs::msgs::sensor_msgs::Image>(
            options_.depth_topic,
            [self](
                const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg) {
                if (msg) {
                    self->OnDepth(msg);
                }
            });
        AINFO << "Atlas ImageBridge: RGBD rgb=" << options_.rgb_topic
              << " depth=" << options_.depth_topic
              << " sync_slop=" << options_.sync_slop_sec << "s"
              << " queue=" << options_.sync_queue_size;
    } else if (cam->setup_type_ == camera::setup_type_t::Monocular) {
        AINFO << "Atlas ImageBridge: monocular rgb=" << options_.rgb_topic;
    } else {
        AERROR << "Atlas ImageBridge: unsupported setup '"
               << cam->get_setup_type_string()
               << "' (only Monocular / RGBD).";
        running_ = false;
        return false;
    }
    return true;
}

void ImageBridge::Stop() {
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    pending_rgb_.clear();
    pending_depth_.clear();
    node_.reset();
}

void ImageBridge::PushFrame(std::deque<BufferedFrame>* queue,
                            BufferedFrame frame) {
    if (queue == nullptr || !frame.valid()) {
        return;
    }
    queue->push_back(std::move(frame));
    while (queue->size() > options_.sync_queue_size) {
        queue->pop_front();
        ++drop_count_;
    }
}

bool ImageBridge::TakeSyncedRgbd(cv::Mat* rgb, cv::Mat* depth,
                                 double* timestamp_sec) {
    if (rgb == nullptr || depth == nullptr || timestamp_sec == nullptr) {
        return false;
    }
    if (pending_rgb_.empty() || pending_depth_.empty()) {
        return false;
    }

    // 1) Exact stamp match (autosim publishes RGB+depth with identical stamps).
    for (std::size_t ri = 0; ri < pending_rgb_.size(); ++ri) {
        for (std::size_t di = 0; di < pending_depth_.size(); ++di) {
            if (pending_rgb_[ri].stamp_ns != pending_depth_[di].stamp_ns) {
                continue;
            }
            *rgb = std::move(pending_rgb_[ri].image);
            *depth = std::move(pending_depth_[di].image);
            *timestamp_sec = pending_rgb_[ri].timestamp_sec;
            pending_rgb_.erase(pending_rgb_.begin(),
                               pending_rgb_.begin() +
                                   static_cast<std::ptrdiff_t>(ri) + 1);
            pending_depth_.erase(pending_depth_.begin(),
                                 pending_depth_.begin() +
                                     static_cast<std::ptrdiff_t>(di) + 1);
            return true;
        }
    }

    // 2) Closest pair within sync_slop (real sensors / network jitter).
    const int64_t slop_ns = static_cast<int64_t>(
        std::llround(options_.sync_slop_sec * 1e9));
    std::size_t best_ri = 0;
    std::size_t best_di = 0;
    int64_t best_dt = std::numeric_limits<int64_t>::max();
    for (std::size_t ri = 0; ri < pending_rgb_.size(); ++ri) {
        for (std::size_t di = 0; di < pending_depth_.size(); ++di) {
            const int64_t dt = std::llabs(pending_rgb_[ri].stamp_ns -
                                          pending_depth_[di].stamp_ns);
            if (dt <= slop_ns && dt < best_dt) {
                best_dt = dt;
                best_ri = ri;
                best_di = di;
            }
        }
    }
    if (best_dt == std::numeric_limits<int64_t>::max()) {
        // Drop the older stream head so queues cannot stall forever.
        if (pending_rgb_.front().stamp_ns < pending_depth_.front().stamp_ns) {
            pending_rgb_.pop_front();
        } else {
            pending_depth_.pop_front();
        }
        ++drop_count_;
        if (drop_count_ == 1 || drop_count_ % 30 == 0) {
            AWARN << "Atlas ImageBridge: RGB/depth sync failed (dropped "
                  << drop_count_ << ", slop=" << options_.sync_slop_sec
                  << "s)";
        }
        return false;
    }

    *rgb = std::move(pending_rgb_[best_ri].image);
    *depth = std::move(pending_depth_[best_di].image);
    // Shared observation time: use the earlier of the two stamps.
    *timestamp_sec = std::min(pending_rgb_[best_ri].timestamp_sec,
                              pending_depth_[best_di].timestamp_sec);
    pending_rgb_.erase(pending_rgb_.begin(),
                       pending_rgb_.begin() +
                           static_cast<std::ptrdiff_t>(best_ri) + 1);
    pending_depth_.erase(pending_depth_.begin(),
                         pending_depth_.begin() +
                             static_cast<std::ptrdiff_t>(best_di) + 1);
    return true;
}

void ImageBridge::FeedMonocular(const cv::Mat& rgb, double timestamp_sec) {
    if (!slam_ || !running_) {
        return;
    }
    const auto cam_pose_wc = slam_->feed_monocular_frame(rgb, timestamp_sec);
    if (viz_) {
        viz_->PublishFrame(timestamp_sec, cam_pose_wc);
    }
    uint64_t fed_snapshot = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++fed_count_;
        fed_snapshot = fed_count_;
    }
    if (fed_snapshot == 1 || fed_snapshot % 100 == 0) {
        AINFO << "Atlas ImageBridge: fed " << fed_snapshot
              << " frames (setup=Monocular).";
    }
}

void ImageBridge::FeedRgbd(const cv::Mat& rgb, const cv::Mat& depth,
                           double timestamp_sec) {
    if (!slam_ || !running_) {
        return;
    }
    const auto cam_pose_wc = slam_->feed_RGBD_frame(rgb, depth, timestamp_sec);
    if (viz_) {
        viz_->PublishFrame(timestamp_sec, cam_pose_wc);
    }
    if (dense_map_) {
        dense_map_->Integrate(rgb, depth, timestamp_sec, cam_pose_wc);
    }
    uint64_t fed_snapshot = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++fed_count_;
        fed_snapshot = fed_count_;
    }
    if (fed_snapshot == 1 || fed_snapshot % 100 == 0) {
        AINFO << "Atlas ImageBridge: fed " << fed_snapshot
              << " frames (setup=RGBD).";
    }
}

void ImageBridge::OnDepth(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg) {
    if (!running_ || !is_rgbd_) {
        return;
    }
    cv::Mat depth;
    if (!ImageMsgToCvMat(*msg, &depth) || !NormalizeDepth(&depth)) {
        return;
    }

    cv::Mat rgb_pair;
    cv::Mat depth_pair;
    double timestamp_sec = 0.0;
    bool ready = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        ++depth_count_;
        BufferedFrame frame;
        frame.image = std::move(depth);
        frame.stamp_ns = StampToNanoseconds(*msg);
        frame.timestamp_sec = StampToSeconds(*msg);
        PushFrame(&pending_depth_, std::move(frame));
        ready = TakeSyncedRgbd(&rgb_pair, &depth_pair, &timestamp_sec);
    }
    if (ready) {
        FeedRgbd(rgb_pair, depth_pair, timestamp_sec);
    }
}

void ImageBridge::OnRgb(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg) {
    if (!running_ || !slam_) {
        return;
    }

    cv::Mat rgb;
    if (!ImageMsgToCvMat(*msg, &rgb)) {
        return;
    }

    const double timestamp = StampToSeconds(*msg);
    const int64_t stamp_ns = StampToNanoseconds(*msg);

    if (!is_rgbd_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_) {
                return;
            }
            ++rgb_count_;
        }
        FeedMonocular(rgb, timestamp);
        return;
    }

    cv::Mat rgb_pair;
    cv::Mat depth_pair;
    double timestamp_sec = 0.0;
    bool ready = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        ++rgb_count_;
        BufferedFrame frame;
        frame.image = std::move(rgb);
        frame.stamp_ns = stamp_ns;
        frame.timestamp_sec = timestamp;
        PushFrame(&pending_rgb_, std::move(frame));
        ready = TakeSyncedRgbd(&rgb_pair, &depth_pair, &timestamp_sec);
        if (!ready && pending_depth_.empty() && (rgb_count_ % 30 == 1)) {
            AWARN << "Atlas ImageBridge: waiting for depth on "
                  << options_.depth_topic;
        }
    }
    if (ready) {
        FeedRgbd(rgb_pair, depth_pair, timestamp_sec);
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
