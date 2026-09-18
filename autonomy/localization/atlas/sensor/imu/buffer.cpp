/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/localization/atlas/sensor/imu/buffer.hpp"

#include <algorithm>

namespace autonomy::localization::atlas {
namespace imu {

buffer::buffer(std::size_t capacity)
    : capacity_(std::max<std::size_t>(1, capacity)) {}

void buffer::set_capacity(std::size_t capacity) {
    std::lock_guard<std::mutex> lock(mtx_);
    capacity_ = std::max<std::size_t>(1, capacity);
    while (data_.size() > capacity_) {
        data_.pop_front();
    }
}

std::size_t buffer::capacity() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return capacity_;
}

std::size_t buffer::size() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_.size();
}

void buffer::clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.clear();
}

void buffer::push(const measurement& m) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data_.empty() && m.timestamp < data_.back().timestamp) {
        // Out-of-order: insert sorted.
        auto it = std::upper_bound(
            data_.begin(), data_.end(), m.timestamp,
            [](double t, const measurement& x) { return t < x.timestamp; });
        data_.insert(it, m);
    } else {
        data_.push_back(m);
    }
    while (data_.size() > capacity_) {
        data_.pop_front();
    }
}

void buffer::push(double timestamp, const Vec3_t& acc, const Vec3_t& gyro) {
    measurement m;
    m.timestamp = timestamp;
    m.a = acc;
    m.w = gyro;
    push(m);
}

eigen_alloc_vector<measurement> buffer::select(double t0, double t1) const {
    std::lock_guard<std::mutex> lock(mtx_);
    eigen_alloc_vector<measurement> out;
    for (const auto& m : data_) {
        if (m.timestamp <= t0) {
            continue;
        }
        if (m.timestamp > t1) {
            break;
        }
        out.push_back(m);
    }
    return out;
}

void buffer::erase_before(double timestamp) {
    std::lock_guard<std::mutex> lock(mtx_);
    while (!data_.empty() && data_.front().timestamp < timestamp) {
        data_.pop_front();
    }
}

bool buffer::empty() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_.empty();
}

measurement buffer::front() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_.front();
}

measurement buffer::back() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_.back();
}

}  // namespace imu
}  // namespace autonomy::localization::atlas
