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

//! Per-voxel storage for IVox: linear ring buffer or PHC (Pseudo Hilbert Curve)
//! sub-cubes. Adapted from lightning-lm / Faster-LIO ivox3d_node (Eigen Vec3_t,
//! no PCL CentroidPoint).

#include "autonomy/localization/atlas/mapping/ivox/hilbert.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

//! Linear voxel node: fixed-capacity ring of points.
class IVoxNodeLinear {
public:
    struct DistPoint {
        double dist2 = 0.0;
        std::size_t idx = 0;
        bool operator<(const DistPoint& o) const { return dist2 < o.dist2; }
    };

    IVoxNodeLinear() = default;
    IVoxNodeLinear(const Vec3_t& /*center*/, double /*side*/,
                   std::size_t max_points)
        : max_points_(std::max<std::size_t>(1, max_points)) {}

    void Insert(const Vec3_t& pt) {
        if (points_.size() >= max_points_) {
            points_.erase(points_.begin());
        }
        points_.push_back(pt);
    }

    [[nodiscard]] bool Empty() const { return points_.empty(); }
    [[nodiscard]] std::size_t Size() const { return points_.size(); }
    [[nodiscard]] const Vec3_t& GetPoint(std::size_t idx) const {
        return points_[idx];
    }

    void CollectAll(std::vector<Vec3_t>* out) const {
        if (!out) {
            return;
        }
        out->insert(out->end(), points_.begin(), points_.end());
    }

    bool NNPoint(const Vec3_t& query, DistPoint* out) const {
        if (!out || points_.empty()) {
            return false;
        }
        DistPoint best;
        best.dist2 = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const double d2 = (points_[i] - query).squaredNorm();
            if (d2 < best.dist2) {
                best.dist2 = d2;
                best.idx = i;
            }
        }
        *out = best;
        return true;
    }

    void KNNByCondition(std::vector<DistPoint>* out, const Vec3_t& query,
                        int K, double max_range) const {
        if (!out || K <= 0) {
            return;
        }
        const double max2 = max_range * max_range;
        const std::size_t old = out->size();
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const double d2 = (points_[i] - query).squaredNorm();
            if (d2 <= max2) {
                out->push_back(DistPoint{d2, i});
            }
        }
        if (out->size() > old + static_cast<std::size_t>(K)) {
            std::nth_element(out->begin() + static_cast<std::ptrdiff_t>(old),
                             out->begin() + static_cast<std::ptrdiff_t>(old) +
                                 K - 1,
                             out->end());
            out->resize(old + static_cast<std::size_t>(K));
        }
    }

private:
    std::vector<Vec3_t> points_;
    std::size_t max_points_ = 20;
};

//! PHC voxel node: subdivide voxel, index by Hilbert, store cube means.
class IVoxNodePhc {
public:
    struct DistPoint {
        double dist2 = 0.0;
        std::size_t idx = 0;
        bool operator<(const DistPoint& o) const { return dist2 < o.dist2; }
    };

    struct PhcCube {
        std::uint32_t idx = 0;
        Vec3_t sum = Vec3_t::Zero();
        int count = 0;

        PhcCube() = default;
        PhcCube(std::uint32_t i, const Vec3_t& pt) : idx(i) { Add(pt); }

        void Add(const Vec3_t& pt) {
            sum += pt;
            ++count;
        }
        [[nodiscard]] Vec3_t Mean() const {
            return count > 0 ? (sum / static_cast<double>(count)) : sum;
        }
    };

    IVoxNodePhc() = default;
    IVoxNodePhc(const Vec3_t& center, double side_length, int phc_order = 6)
        : center_(center),
          side_length_(static_cast<float>(side_length)),
          phc_order_(std::clamp(phc_order, 1, 8)) {
        const float n = static_cast<float>(std::pow(2.0, phc_order_));
        phc_side_length_ = side_length_ / n;
        phc_side_length_inv_ = n / side_length_;
        min_cube_ = center_.cast<float>() -
                    Eigen::Vector3f::Constant(side_length_ * 0.5f);
        phc_cubes_.reserve(64);
    }

    void Insert(const Vec3_t& pt) {
        const std::uint32_t idx = CalculatePhcIndex(pt);
        PhcCube cube{idx, pt};
        auto it = std::lower_bound(
            phc_cubes_.begin(), phc_cubes_.end(), cube,
            [](const PhcCube& a, const PhcCube& b) { return a.idx < b.idx; });
        if (it == phc_cubes_.end()) {
            phc_cubes_.push_back(cube);
        } else if (it->idx == idx) {
            it->Add(pt);
        } else {
            phc_cubes_.insert(it, cube);
        }
    }

    [[nodiscard]] bool Empty() const { return phc_cubes_.empty(); }
    [[nodiscard]] std::size_t Size() const { return phc_cubes_.size(); }
    [[nodiscard]] Vec3_t GetPoint(std::size_t idx) const {
        return phc_cubes_[idx].Mean();
    }

    void CollectAll(std::vector<Vec3_t>* out) const {
        if (!out) {
            return;
        }
        for (const auto& c : phc_cubes_) {
            out->push_back(c.Mean());
        }
    }

    bool NNPoint(const Vec3_t& query, DistPoint* out) const {
        if (!out || phc_cubes_.empty()) {
            return false;
        }
        const std::uint32_t cur = CalculatePhcIndex(query);
        PhcCube probe;
        probe.idx = cur;
        auto it = std::lower_bound(
            phc_cubes_.begin(), phc_cubes_.end(), probe,
            [](const PhcCube& a, const PhcCube& b) { return a.idx < b.idx; });
        if (it == phc_cubes_.end()) {
            --it;
        } else if (it != phc_cubes_.begin()) {
            auto prev = it;
            --prev;
            const double d1 = (it->Mean() - query).squaredNorm();
            const double d2 = (prev->Mean() - query).squaredNorm();
            if (d2 < d1) {
                it = prev;
            }
        }
        out->dist2 = (it->Mean() - query).squaredNorm();
        out->idx = static_cast<std::size_t>(it - phc_cubes_.begin());
        return true;
    }

    void KNNByCondition(std::vector<DistPoint>* out, const Vec3_t& query,
                        int K, double max_range) const {
        if (!out || K <= 0 || phc_cubes_.empty()) {
            return;
        }
        const std::uint32_t cur = CalculatePhcIndex(query);
        const double max2 = max_range * max_range;
        const int max_side = static_cast<int>(
            std::pow(2.0, std::ceil(std::log2(
                              std::max(1.0, max_range * phc_side_length_inv_)))));
        const int max_idx_th = 8 * max_side * max_side * max_side;

        const std::size_t old = out->size();
        for (std::size_t i = 0; i < phc_cubes_.size(); ++i) {
            const auto& c = phc_cubes_[i];
            if (std::abs(static_cast<int>(c.idx) - static_cast<int>(cur)) >
                max_idx_th) {
                continue;
            }
            const double d2 = (c.Mean() - query).squaredNorm();
            if (d2 <= max2) {
                out->push_back(DistPoint{d2, i});
            }
        }
        if (out->size() > old + static_cast<std::size_t>(K)) {
            std::nth_element(out->begin() + static_cast<std::ptrdiff_t>(old),
                             out->begin() + static_cast<std::ptrdiff_t>(old) +
                                 K - 1,
                             out->end());
            out->resize(old + static_cast<std::size_t>(K));
        }
    }

private:
    [[nodiscard]] std::uint32_t CalculatePhcIndex(const Vec3_t& pt) const {
        Eigen::Vector3f eposf =
            (pt.cast<float>() - min_cube_) * phc_side_length_inv_;
        Eigen::Vector3i eposi = eposf.array().floor().cast<int>();
        const int lim = static_cast<int>(std::pow(2.0, phc_order_)) - 1;
        for (int i = 0; i < 3; ++i) {
            eposi(i) = std::clamp(eposi(i), 0, lim);
        }
        const std::array<std::uint8_t, 3> apos{
            static_cast<std::uint8_t>(eposi(0)),
            static_cast<std::uint8_t>(eposi(1)),
            static_cast<std::uint8_t>(eposi(2))};
        const std::array<std::uint8_t, 3> tmp =
            hilbert::v2::PositionToIndex(apos);
        return (static_cast<std::uint32_t>(tmp[0]) << 16) |
               (static_cast<std::uint32_t>(tmp[1]) << 8) |
               static_cast<std::uint32_t>(tmp[2]);
    }

    Vec3_t center_ = Vec3_t::Zero();
    float side_length_ = 0.f;
    int phc_order_ = 6;
    float phc_side_length_ = 0.f;
    float phc_side_length_inv_ = 0.f;
    Eigen::Vector3f min_cube_ = Eigen::Vector3f::Zero();
    std::vector<PhcCube> phc_cubes_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
