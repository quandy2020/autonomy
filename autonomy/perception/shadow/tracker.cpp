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
 * @brief Two-pass IoU association and selected-target lifecycle for Shadow.
 */

#include "autonomy/perception/shadow/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
using Detection2DArray = automsgs::msgs::vision_msgs::Detection2DArray;

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr double kMeasurementNoise = 1.0;
constexpr double kPositionProcessNoise = 0.25;
constexpr double kVelocityProcessNoise = 1.0;

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow tracker: " + message;
    }
}

int64_t SecondsToNanoseconds(float seconds) {
    const double nanoseconds =
        static_cast<double>(seconds) * kNanosecondsPerSecond;
    if (!std::isfinite(nanoseconds) || nanoseconds <= 0.0) {
        return 0;
    }
    if (nanoseconds >=
        static_cast<double>(std::numeric_limits<int64_t>::max())) {
        return std::numeric_limits<int64_t>::max();
    }
    return static_cast<int64_t>(nanoseconds);
}

int64_t ElapsedNanoseconds(int64_t now_ns, int64_t then_ns) {
    return now_ns > then_ns ? now_ns - then_ns : 0;
}

struct Box {
    double center_x = 0.0;
    double center_y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

bool DetectionBox(const Detection2D& detection, Box* box) {
    if (box == nullptr) {
        return false;
    }
    const auto& bbox = detection.bbox();
    const double center_x = bbox.center().position().x();
    const double center_y = bbox.center().position().y();
    const double width = bbox.size_x();
    const double height = bbox.size_y();
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
        !std::isfinite(width) || !std::isfinite(height) || width <= 0.0 ||
        height <= 0.0) {
        return false;
    }
    *box = {center_x, center_y, width, height};
    return true;
}

double DetectionConfidence(const Detection2D& detection) {
    double confidence = -std::numeric_limits<double>::infinity();
    for (const auto& result : detection.results()) {
        const double score = result.hypothesis().score();
        if (std::isfinite(score)) {
            confidence = std::max(confidence, score);
        }
    }
    return confidence;
}

double IntersectionOverUnion(const Box& lhs, const Box& rhs) {
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
    return union_area > 0.0 ? intersection / union_area : 0.0;
}

class KalmanCoordinate
{
public:
    explicit KalmanCoordinate(double value) : value_(value) {}

    void Predict(double dt_sec) {
        if (dt_sec <= 0.0) {
            return;
        }
        value_ += velocity_ * dt_sec;

        const double p00 = p00_ + dt_sec * (p10_ + p01_) +
                           dt_sec * dt_sec * p11_ + kPositionProcessNoise;
        const double p01 = p01_ + dt_sec * p11_;
        const double p10 = p10_ + dt_sec * p11_;
        const double p11 = p11_ + kVelocityProcessNoise;
        p00_ = p00;
        p01_ = p01;
        p10_ = p10;
        p11_ = p11;
    }

    void Correct(double measurement) {
        const double innovation_covariance = p00_ + kMeasurementNoise;
        const double position_gain = p00_ / innovation_covariance;
        const double velocity_gain = p10_ / innovation_covariance;
        const double innovation = measurement - value_;
        value_ += position_gain * innovation;
        velocity_ += velocity_gain * innovation;

        const double p00 = (1.0 - position_gain) * p00_;
        const double p01 = (1.0 - position_gain) * p01_;
        const double p10 = p10_ - velocity_gain * p00_;
        const double p11 = p11_ - velocity_gain * p01_;
        p00_ = p00;
        p01_ = p01;
        p10_ = p10;
        p11_ = p11;
    }

    double value() const {
        return value_;
    }

private:
    double value_ = 0.0;
    double velocity_ = 0.0;
    double p00_ = 10.0;
    double p01_ = 0.0;
    double p10_ = 0.0;
    double p11_ = 100.0;
};

class KalmanBox
{
public:
    explicit KalmanBox(const Box& box)
        : center_x_(box.center_x),
          center_y_(box.center_y),
          width_(box.width),
          height_(box.height) {}

    void Predict(double dt_sec) {
        center_x_.Predict(dt_sec);
        center_y_.Predict(dt_sec);
        width_.Predict(dt_sec);
        height_.Predict(dt_sec);
    }

    void Correct(const Box& box) {
        center_x_.Correct(box.center_x);
        center_y_.Correct(box.center_y);
        width_.Correct(box.width);
        height_.Correct(box.height);
    }

    Box box() const {
        return {center_x_.value(), center_y_.value(),
                std::max(0.0, width_.value()), std::max(0.0, height_.value())};
    }

private:
    KalmanCoordinate center_x_;
    KalmanCoordinate center_y_;
    KalmanCoordinate width_;
    KalmanCoordinate height_;
};

void SetDetectionBox(const Box& box, Detection2D* detection) {
    auto* bbox = detection->mutable_bbox();
    bbox->mutable_center()->mutable_position()->set_x(box.center_x);
    bbox->mutable_center()->mutable_position()->set_y(box.center_y);
    bbox->set_size_x(box.width);
    bbox->set_size_y(box.height);
}

}  // namespace

struct PersonTracker::Impl {
    struct Track {
        Track(uint64_t track_id, const Box& box, const Detection2D& detection,
              int64_t stamp_ns, uint32_t minimum_hits)
            : id(track_id),
              filter(box),
              output(detection),
              confirmed(minimum_hits <= 1),
              last_seen_ns(stamp_ns),
              state_stamp_ns(stamp_ns) {}

        uint64_t id;
        KalmanBox filter;
        Detection2D output;
        uint32_t hits = 1;
        bool confirmed = false;
        bool visible = true;
        int64_t last_seen_ns;
        int64_t state_stamp_ns;
    };

    struct AssociationCandidate {
        double iou;
        size_t track_index;
        int detection_index;
        uint64_t track_id;
    };

    explicit Impl(const proto::ShadowOptions& source_options)
        : options(source_options),
          prediction_timeout_ns(
              SecondsToNanoseconds(options.prediction_timeout_sec())),
          lost_timeout_ns(SecondsToNanoseconds(options.lost_timeout_sec())) {}

    void Associate(const std::vector<int>& detection_indices,
                   const Detection2DArray& detections, bool confirmed_only,
                   std::vector<bool>* matched_tracks,
                   std::vector<bool>* matched_detections,
                   std::vector<std::pair<size_t, int>>* matches) {
        std::vector<AssociationCandidate> candidates;
        for (size_t track_index = 0; track_index < tracks.size();
             ++track_index) {
            if ((*matched_tracks)[track_index] ||
                (confirmed_only && !tracks[track_index].confirmed)) {
                continue;
            }
            const Box track_box = tracks[track_index].filter.box();
            for (const int detection_index : detection_indices) {
                if ((*matched_detections)[static_cast<size_t>(
                        detection_index)]) {
                    continue;
                }
                Box detection_box;
                if (!DetectionBox(detections.detections(detection_index),
                                  &detection_box)) {
                    continue;
                }
                const double iou =
                    IntersectionOverUnion(track_box, detection_box);
                if (iou >= options.association_iou_threshold()) {
                    candidates.push_back({iou, track_index, detection_index,
                                          tracks[track_index].id});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const AssociationCandidate& lhs,
                     const AssociationCandidate& rhs) {
                      if (lhs.iou != rhs.iou) {
                          return lhs.iou > rhs.iou;
                      }
                      if (lhs.track_id != rhs.track_id) {
                          return lhs.track_id < rhs.track_id;
                      }
                      return lhs.detection_index < rhs.detection_index;
                  });
        for (const auto& candidate : candidates) {
            const size_t detection_index =
                static_cast<size_t>(candidate.detection_index);
            if ((*matched_tracks)[candidate.track_index] ||
                (*matched_detections)[detection_index]) {
                continue;
            }
            (*matched_tracks)[candidate.track_index] = true;
            (*matched_detections)[detection_index] = true;
            matches->emplace_back(candidate.track_index,
                                  candidate.detection_index);
        }
    }

    void ApplyMatch(int64_t stamp_ns, int detection_index,
                    Detection2DArray* detections, Track* track) {
        auto* detection = detections->mutable_detections(detection_index);
        Box measured_box;
        DetectionBox(*detection, &measured_box);
        track->filter.Correct(measured_box);
        detection->set_id(std::to_string(track->id));
        track->output = *detection;
        track->last_seen_ns = stamp_ns;
        ++track->hits;
        track->confirmed =
            track->confirmed || track->hits >= options.min_confirmed_hits();
        track->visible = true;
    }

    const Track* FindSelected() const {
        if (selected_id.empty()) {
            return nullptr;
        }
        for (const auto& track : tracks) {
            if (std::to_string(track.id) == selected_id) {
                return &track;
            }
        }
        return nullptr;
    }

    bool IsPredicted(const Track& track) const {
        return !track.visible && prediction_timeout_ns > 0 &&
               ElapsedNanoseconds(latest_stamp_ns, track.last_seen_ns) <=
                   prediction_timeout_ns;
    }

    proto::ShadowOptions options;
    std::vector<Track> tracks;
    std::string selected_id;
    uint64_t next_id = 1;
    int64_t latest_stamp_ns = 0;
    int64_t prediction_timeout_ns;
    int64_t lost_timeout_ns;
    automsgs::msgs::std_msgs::Header latest_header;
};

PersonTracker::PersonTracker(const proto::ShadowOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}

PersonTracker::~PersonTracker() = default;

bool PersonTracker::Update(int64_t stamp_ns, Detection2DArray* detections,
                           std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (detections == nullptr) {
        SetError(error, "detections are null.");
        return false;
    }
    if (stamp_ns < impl_->latest_stamp_ns) {
        SetError(error, "timestamp precedes the previous frame.");
        return false;
    }

    impl_->latest_stamp_ns = stamp_ns;
    impl_->latest_header = detections->header();
    for (auto& track : impl_->tracks) {
        track.visible = false;
        const double dt_sec = static_cast<double>(ElapsedNanoseconds(
                                  stamp_ns, track.state_stamp_ns)) /
                              kNanosecondsPerSecond;
        track.filter.Predict(dt_sec);
        track.state_stamp_ns = stamp_ns;
    }
    impl_->tracks.erase(
        std::remove_if(impl_->tracks.begin(), impl_->tracks.end(),
                       [this, stamp_ns](const Impl::Track& track) {
                           return impl_->lost_timeout_ns > 0 &&
                                  ElapsedNanoseconds(stamp_ns,
                                                     track.last_seen_ns) >
                                      impl_->lost_timeout_ns;
                       }),
        impl_->tracks.end());

    std::vector<int> high_confidence;
    std::vector<int> low_confidence;
    std::vector<bool> valid_detection(
        static_cast<size_t>(detections->detections_size()), false);
    for (int index = 0; index < detections->detections_size(); ++index) {
        auto* detection = detections->mutable_detections(index);
        detection->clear_id();
        Box box;
        if (!DetectionBox(*detection, &box)) {
            continue;
        }
        const double confidence = DetectionConfidence(*detection);
        if (confidence >= impl_->options.track_high_threshold()) {
            high_confidence.push_back(index);
            valid_detection[static_cast<size_t>(index)] = true;
        } else if (confidence >= impl_->options.track_low_threshold()) {
            low_confidence.push_back(index);
            valid_detection[static_cast<size_t>(index)] = true;
        }
    }

    std::vector<bool> matched_tracks(impl_->tracks.size(), false);
    std::vector<bool> matched_detections(
        static_cast<size_t>(detections->detections_size()), false);
    std::vector<std::pair<size_t, int>> high_matches;
    impl_->Associate(high_confidence, *detections, false, &matched_tracks,
                     &matched_detections, &high_matches);
    for (const auto& match : high_matches) {
        impl_->ApplyMatch(stamp_ns, match.second, detections,
                          &impl_->tracks[match.first]);
    }

    std::vector<std::pair<size_t, int>> low_matches;
    impl_->Associate(low_confidence, *detections, true, &matched_tracks,
                     &matched_detections, &low_matches);
    for (const auto& match : low_matches) {
        impl_->ApplyMatch(stamp_ns, match.second, detections,
                          &impl_->tracks[match.first]);
    }

    for (const int detection_index : high_confidence) {
        if (matched_detections[static_cast<size_t>(detection_index)] ||
            !valid_detection[static_cast<size_t>(detection_index)]) {
            continue;
        }
        auto* detection = detections->mutable_detections(detection_index);
        Box box;
        DetectionBox(*detection, &box);
        const uint64_t id = impl_->next_id++;
        detection->set_id(std::to_string(id));
        impl_->tracks.emplace_back(id, box, *detection, stamp_ns,
                                   impl_->options.min_confirmed_hits());
    }
    return true;
}

void PersonTracker::Select(const std::string& target_id) {
    impl_->selected_id = target_id;
}

bool PersonTracker::Selected(Detection2D* detection) const {
    const Impl::Track* track = impl_->FindSelected();
    if (track == nullptr || (!track->visible && !impl_->IsPredicted(*track))) {
        return false;
    }
    if (detection != nullptr) {
        *detection = track->output;
        detection->set_id(std::to_string(track->id));
        if (!track->visible) {
            SetDetectionBox(track->filter.box(), detection);
        }
    }
    return true;
}

void PersonTracker::Confirmed(Detection2DArray* detections) const {
    if (detections == nullptr) {
        return;
    }
    detections->Clear();
    *detections->mutable_header() = impl_->latest_header;
    for (const auto& track : impl_->tracks) {
        if (track.confirmed && track.visible) {
            *detections->add_detections() = track.output;
        }
    }
}

bool PersonTracker::selected_visible() const {
    const Impl::Track* track = impl_->FindSelected();
    return track != nullptr && track->visible;
}

bool PersonTracker::selected_predicted() const {
    const Impl::Track* track = impl_->FindSelected();
    return track != nullptr && impl_->IsPredicted(*track);
}

void PersonTracker::Clear() {
    impl_->tracks.clear();
    impl_->selected_id.clear();
    impl_->latest_stamp_ns = 0;
    impl_->latest_header.Clear();
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
