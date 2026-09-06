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
 * @file tracker.hpp
 * @brief Multi-class IoU track association.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_TRACKER_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_TRACKER_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {

class Tracker
{
public:
    explicit Tracker(const proto::HestiaOptions& options) : options_(options) {}

    void Associate(double stamp_sec,
                   automsgs::msgs::vision_msgs::Detection2DArray* detections) {
        if (detections == nullptr) {
            return;
        }

        const double timeout = options_.lost_timeout_sec();
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                     [&](const Track& track) {
                                         return stamp_sec - track.last_stamp_sec >
                                                timeout;
                                     }),
                      tracks_.end());

        std::vector<bool> track_used(tracks_.size(), false);

        for (auto& detection : *detections->mutable_detections()) {
            double center_x = 0.0;
            double center_y = 0.0;
            double width = 0.0;
            double height = 0.0;
            std::string class_id;
            if (!ReadBox(detection, &center_x, &center_y, &width, &height,
                         &class_id)) {
                detection.clear_id();
                continue;
            }

            int best_index = -1;
            double best_iou = options_.association_iou_threshold();
            for (size_t index = 0; index < tracks_.size(); ++index) {
                if (track_used[index]) {
                    continue;
                }
                const Track& track = tracks_[index];
                if (!class_id.empty() && !track.class_id.empty() &&
                    class_id != track.class_id) {
                    continue;
                }
                const double iou =
                    IoU(center_x, center_y, width, height, track.center_x,
                        track.center_y, track.width, track.height);
                if (iou < options_.association_iou_threshold()) {
                    continue;
                }
                if (best_index < 0 || iou > best_iou + 1e-12) {
                    best_iou = iou;
                    best_index = static_cast<int>(index);
                }
            }

            if (best_index >= 0) {
                Track& track = tracks_[static_cast<size_t>(best_index)];
                track_used[static_cast<size_t>(best_index)] = true;
                track.center_x = center_x;
                track.center_y = center_y;
                track.width = width;
                track.height = height;
                track.last_stamp_sec = stamp_sec;
                if (!class_id.empty()) {
                    track.class_id = class_id;
                }
                detection.set_id(track.id);
                continue;
            }

            Track track;
            track.id = std::to_string(next_id_++);
            track.class_id = class_id;
            track.center_x = center_x;
            track.center_y = center_y;
            track.width = width;
            track.height = height;
            track.last_stamp_sec = stamp_sec;
            detection.set_id(track.id);
            tracks_.push_back(std::move(track));
            track_used.push_back(true);
        }
    }

    void Clear() { tracks_.clear(); }

private:
    struct Track {
        std::string id;
        std::string class_id;
        double center_x = 0.0;
        double center_y = 0.0;
        double width = 0.0;
        double height = 0.0;
        double last_stamp_sec = 0.0;
    };

    static bool ReadBox(
        const automsgs::msgs::vision_msgs::Detection2D& detection,
        double* center_x, double* center_y, double* width, double* height,
        std::string* class_id) {
        const auto& bbox = detection.bbox();
        *center_x = bbox.center().position().x();
        *center_y = bbox.center().position().y();
        *width = bbox.size_x();
        *height = bbox.size_y();
        if (!std::isfinite(*center_x) || !std::isfinite(*center_y) ||
            !std::isfinite(*width) || !std::isfinite(*height) || *width <= 0.0 ||
            *height <= 0.0) {
            return false;
        }
        *class_id = detection.results().empty()
                        ? std::string()
                        : detection.results(0).hypothesis().class_id();
        return true;
    }

    static double IoU(double ax, double ay, double aw, double ah, double bx,
                      double by, double bw, double bh) {
        const double a_left = ax - aw * 0.5;
        const double a_top = ay - ah * 0.5;
        const double a_right = ax + aw * 0.5;
        const double a_bottom = ay + ah * 0.5;
        const double b_left = bx - bw * 0.5;
        const double b_top = by - bh * 0.5;
        const double b_right = bx + bw * 0.5;
        const double b_bottom = by + bh * 0.5;
        const double iw =
            std::max(0.0, std::min(a_right, b_right) - std::max(a_left, b_left));
        const double ih =
            std::max(0.0, std::min(a_bottom, b_bottom) - std::max(a_top, b_top));
        const double intersection = iw * ih;
        const double union_area = aw * ah + bw * bh - intersection;
        return union_area > 0.0 ? intersection / union_area : 0.0;
    }

    proto::HestiaOptions options_;
    std::vector<Track> tracks_;
    uint64_t next_id_ = 1;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_TRACKER_HPP_
