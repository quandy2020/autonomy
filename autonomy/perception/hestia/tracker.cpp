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
 * @file tracker.cpp
 * @brief Greedy IoU association for multi-class Hestia tracks.
 */

#include "autonomy/perception/hestia/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

struct Box {
    double center_x = 0.0;
    double center_y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

bool ReadBox(const automsgs::msgs::vision_msgs::Detection2D& detection,
             Box* box, std::string* class_id) {
    const auto& bbox = detection.bbox();
    box->center_x = bbox.center().position().x();
    box->center_y = bbox.center().position().y();
    box->width = bbox.size_x();
    box->height = bbox.size_y();
    if (!std::isfinite(box->center_x) || !std::isfinite(box->center_y) ||
        !std::isfinite(box->width) || !std::isfinite(box->height) ||
        box->width <= 0.0 || box->height <= 0.0) {
        return false;
    }
    *class_id = detection.results().empty()
                    ? std::string()
                    : detection.results(0).hypothesis().class_id();
    return true;
}

double IoU(const Box& lhs, const Box& rhs) {
    const double lhs_left = lhs.center_x - lhs.width * 0.5;
    const double lhs_top = lhs.center_y - lhs.height * 0.5;
    const double lhs_right = lhs.center_x + lhs.width * 0.5;
    const double lhs_bottom = lhs.center_y + lhs.height * 0.5;
    const double rhs_left = rhs.center_x - rhs.width * 0.5;
    const double rhs_top = rhs.center_y - rhs.height * 0.5;
    const double rhs_right = rhs.center_x + rhs.width * 0.5;
    const double rhs_bottom = rhs.center_y + rhs.height * 0.5;
    const double intersection_width = std::max(
        0.0, std::min(lhs_right, rhs_right) - std::max(lhs_left, rhs_left));
    const double intersection_height = std::max(
        0.0, std::min(lhs_bottom, rhs_bottom) - std::max(lhs_top, rhs_top));
    const double intersection = intersection_width * intersection_height;
    const double union_area =
        lhs.width * lhs.height + rhs.width * rhs.height - intersection;
    if (union_area <= 0.0) {
        return 0.0;
    }
    return intersection / union_area;
}

}  // namespace

ObjectTracker::ObjectTracker(const proto::HestiaOptions& options)
    : options_(options) {}

void ObjectTracker::Clear() { tracks_.clear(); }

void ObjectTracker::Update(
    double stamp_sec,
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
        Box box;
        std::string class_id;
        if (!ReadBox(detection, &box, &class_id)) {
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
            const Box track_box{track.center_x, track.center_y, track.width,
                                track.height};
            const double iou = IoU(box, track_box);
            if (iou < options_.association_iou_threshold()) {
                continue;
            }
            const bool better_iou = iou > best_iou + 1e-12;
            const bool tied_iou = std::abs(iou - best_iou) <= 1e-12;
            const bool prefer_label =
                best_index >= 0 && !class_id.empty() &&
                class_id == track.class_id &&
                class_id != tracks_[static_cast<size_t>(best_index)].class_id;
            if (best_index < 0 || better_iou || (tied_iou && prefer_label)) {
                best_iou = iou;
                best_index = static_cast<int>(index);
            }
        }

        if (best_index >= 0) {
            Track& track = tracks_[static_cast<size_t>(best_index)];
            track_used[static_cast<size_t>(best_index)] = true;
            track.center_x = box.center_x;
            track.center_y = box.center_y;
            track.width = box.width;
            track.height = box.height;
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
        track.center_x = box.center_x;
        track.center_y = box.center_y;
        track.width = box.width;
        track.height = box.height;
        track.last_stamp_sec = stamp_sec;
        detection.set_id(track.id);
        tracks_.push_back(std::move(track));
        track_used.push_back(true);
    }
}

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
