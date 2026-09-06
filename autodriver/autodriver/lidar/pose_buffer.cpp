/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/lidar/pose_buffer.hpp"

#include <algorithm>

namespace autodriver {
namespace lidar {

PoseBuffer::PoseBuffer(std::size_t capacity) : capacity_(capacity) {
    if (capacity_ == 0) {
        capacity_ = 1;
    }
}

void PoseBuffer::Push(std::uint64_t time_ns, const Eigen::Affine3d& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    poses_.emplace_back(time_ns, pose);
    while (poses_.size() > capacity_) {
        poses_.pop_front();
    }
}

void PoseBuffer::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    poses_.clear();
}

bool PoseBuffer::LookupUnlocked(std::uint64_t time_ns,
                                Eigen::Affine3d* pose) const {
    if (pose == nullptr || poses_.empty()) {
        return false;
    }
    if (poses_.size() == 1) {
        *pose = poses_.front().second;
        return true;
    }
    if (time_ns <= poses_.front().first) {
        *pose = poses_.front().second;
        return true;
    }
    if (time_ns >= poses_.back().first) {
        *pose = poses_.back().second;
        return true;
    }
    for (std::size_t i = 1; i < poses_.size(); ++i) {
        if (time_ns <= poses_[i].first) {
            const auto& a = poses_[i - 1];
            const auto& b = poses_[i];
            const double alpha =
                static_cast<double>(time_ns - a.first) /
                static_cast<double>(b.first - a.first);
            Eigen::Quaterniond qa(a.second.linear());
            Eigen::Quaterniond qb(b.second.linear());
            Eigen::Quaterniond q = qa.slerp(alpha, qb);
            Eigen::Vector3d p =
                (1.0 - alpha) * a.second.translation() +
                alpha * b.second.translation();
            *pose = Eigen::Translation3d(p) * q;
            return true;
        }
    }
    return false;
}

PoseLookup PoseBuffer::AsLookup() const {
    return [this](std::uint64_t time_ns, const std::string&,
                  Eigen::Affine3d* pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        return LookupUnlocked(time_ns, pose);
    };
}

}  // namespace lidar
}  // namespace autodriver
