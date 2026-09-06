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
 * @file merger.hpp
 * @brief Late fusion of closed-set and open-vocabulary detections.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_MERGER_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_MERGER_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

using Detection2D = automsgs::msgs::vision_msgs::Detection2D;

inline double ScoreOf(const Detection2D& detection) {
    double score = 0.0;
    for (const auto& result : detection.results()) {
        score = std::max(score, result.hypothesis().score());
    }
    return score;
}

inline double DetectionIoU(const Detection2D& lhs, const Detection2D& rhs) {
    const auto& a = lhs.bbox();
    const auto& b = rhs.bbox();
    const double a_left = a.center().position().x() - a.size_x() * 0.5;
    const double a_top = a.center().position().y() - a.size_y() * 0.5;
    const double a_right = a.center().position().x() + a.size_x() * 0.5;
    const double a_bottom = a.center().position().y() + a.size_y() * 0.5;
    const double b_left = b.center().position().x() - b.size_x() * 0.5;
    const double b_top = b.center().position().y() - b.size_y() * 0.5;
    const double b_right = b.center().position().x() + b.size_x() * 0.5;
    const double b_bottom = b.center().position().y() + b.size_y() * 0.5;
    const double iw =
        std::max(0.0, std::min(a_right, b_right) - std::max(a_left, b_left));
    const double ih =
        std::max(0.0, std::min(a_bottom, b_bottom) - std::max(a_top, b_top));
    const double intersection = iw * ih;
    const double union_area =
        a.size_x() * a.size_y() + b.size_x() * b.size_y() - intersection;
    return union_area > 0.0 ? intersection / union_area : 0.0;
}

}  // namespace

class Merger
{
public:
    explicit Merger(const proto::HestiaOptions& options) : options_(options) {}

    void Fuse(const automsgs::msgs::vision_msgs::Detection2DArray& closed_set,
              const automsgs::msgs::vision_msgs::Detection2DArray& open_vocab,
              automsgs::msgs::vision_msgs::Detection2DArray* out) const {
        if (out == nullptr) {
            return;
        }
        out->Clear();
        if (closed_set.detections_size() > 0) {
            *out->mutable_header() = closed_set.header();
        } else {
            *out->mutable_header() = open_vocab.header();
        }

        std::vector<Detection2D> candidates;
        candidates.reserve(static_cast<size_t>(closed_set.detections_size() +
                                               open_vocab.detections_size()));
        for (const auto& detection : closed_set.detections()) {
            candidates.push_back(detection);
        }
        for (const auto& detection : open_vocab.detections()) {
            candidates.push_back(detection);
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Detection2D& lhs, const Detection2D& rhs) {
                      return ScoreOf(lhs) > ScoreOf(rhs);
                  });

        std::vector<bool> suppressed(candidates.size(), false);
        const double threshold = options_.merge_iou_threshold();
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (suppressed[i]) {
                continue;
            }
            *out->add_detections() = candidates[i];
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                if (!suppressed[j] &&
                    DetectionIoU(candidates[i], candidates[j]) >= threshold) {
                    suppressed[j] = true;
                }
            }
        }
    }

private:
    proto::HestiaOptions options_;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_MERGER_HPP_
