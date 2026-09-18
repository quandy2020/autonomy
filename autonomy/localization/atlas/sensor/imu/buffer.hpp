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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IMU_BUFFER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IMU_BUFFER_HPP_

#include "autonomy/localization/atlas/sensor/imu/measurement.hpp"

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

namespace autonomy::localization::atlas {
namespace imu {

//! Thread-safe chronologically ordered IMU sample buffer.
class buffer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit buffer(std::size_t capacity = 10000);

    void set_capacity(std::size_t capacity);
    std::size_t capacity() const;
    std::size_t size() const;
    void clear();

    void push(const measurement& m);
    void push(double timestamp, const Vec3_t& acc, const Vec3_t& gyro);

    //! Measurements with timestamps in (t0, t1], sorted ascending.
    eigen_alloc_vector<measurement> select(double t0, double t1) const;

    //! Drop samples older than `timestamp`.
    void erase_before(double timestamp);

    bool empty() const;
    measurement front() const;
    measurement back() const;

private:
    mutable std::mutex mtx_;
    std::size_t capacity_;
    std::deque<measurement> data_;
};

}  // namespace imu
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IMU_BUFFER_HPP_
