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

#include "autonomy/perception/behaviors/tracking/association.hpp"

#include "autonomy/perception/tasks/detection/detection_task.hpp"

#include <algorithm>
#include <vector>

namespace autonomy {
namespace perception {
namespace base {
namespace track {

namespace {
using Box = automsgs::msgs::vision_msgs::BoundingBox2D;

double Overlap(const Box& a, const Box& b) {
    const auto& p = a.center().position();
    const auto& q = b.center().position();
    const double width = std::min(p.x() + a.size_x() / 2, q.x() + b.size_x() / 2) -
                         std::max(p.x() - a.size_x() / 2, q.x() - b.size_x() / 2);
    const double height = std::min(p.y() + a.size_y() / 2, q.y() + b.size_y() / 2) -
                          std::max(p.y() - a.size_y() / 2, q.y() - b.size_y() / 2);
    if (width <= 0 || height <= 0)
        return 0;
    const double intersection = width * height;
    return intersection / (a.size_x() * a.size_y() + b.size_x() * b.size_y() -
                           intersection);
}
}  // namespace

void Associator::Update(
    const automsgs::msgs::vision_msgs::Detection2DArray& detections,
    automsgs::msgs::vision_msgs::Detection2DArray* tracks) {
    tracks->Clear();
    tracks->mutable_header()->CopyFrom(detections.header());
    for (const auto& detection : detections.detections()) {
        if (detection.results_size() && detection.results(0).hypothesis().class_id() == "person")
            tracks->add_detections()->CopyFrom(detection);
    }

    // shortcut: adjacent-frame IoU only, add appearance matching if occlusion
    // recovery is required. Ambiguous matches deliberately lose the old ID.
    std::vector<int> matches(tracks->detections_size(), -1);
    std::vector<int> claims(previous_.detections_size(), 0);
    const auto seconds = [](const auto& stamp) {
        return static_cast<double>(stamp.sec()) + 1e-9 * stamp.nanosec();
    };
    const double dt = seconds(tracks->header().stamp()) -
                      seconds(previous_.header().stamp());
    if (dt > 0 && dt <= 0.5 &&
        tracks->header().frame_id() == previous_.header().frame_id()) {
        for (int i = 0; i < tracks->detections_size(); ++i) {
            int candidates = 0;
            for (int j = 0; j < previous_.detections_size(); ++j) {
                if (Overlap(tracks->detections(i).bbox(),
                            previous_.detections(j).bbox()) < 0.3)
                    continue;
                ++candidates;
                ++claims[j];
                matches[i] = j;
            }
            if (candidates != 1)
                matches[i] = -1;
        }
    }
    for (int i = 0; i < tracks->detections_size(); ++i) {
        const int match = matches[i];
        tracks->mutable_detections(i)->set_id(
            match >= 0 && claims[match] == 1
                ? previous_.detections(match).id()
                : std::to_string(next_id_++));
    }
    previous_.CopyFrom(*tracks);
}

bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb,
            Associator* associator, Outputs* outputs, std::string* error) {
    Outputs frame;
    if (!detect::Decode(engine, options, rgb, &frame, error, proto::TASK_TRACK))
        return false;
    associator->Update(frame.detections, &outputs->tracks);
    return true;
}

}  // namespace track
}  // namespace base
}  // namespace perception
}  // namespace autonomy
