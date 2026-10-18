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

/**
 * @file point_types.hpp
 * @brief PCL-free point structs + PointCloud container (no protobuf).
 */

#ifndef ARGUS_SENSOR_POINT_TYPES_HPP_
#define ARGUS_SENSOR_POINT_TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "argus/common/macros.hpp"

namespace argus {
namespace sensor {

/** @brief Minimal LiDAR point: XYZ + reflectivity. */
struct EIGEN_ALIGN16 PointXYZI {
    float x = 0.f;         /**< @brief X coordinate (m). */
    float y = 0.f;         /**< @brief Y coordinate (m). */
    float z = 0.f;         /**< @brief Z coordinate (m). */
    float intensity = 0.f; /**< @brief Reflectivity or normalized intensity. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief LIVO point: XYZ, intensity, optional normal, relative time in
 * curvature. */
struct EIGEN_ALIGN16 PointXYZINormal {
    float x = 0.f;         /**< @brief X coordinate (m). */
    float y = 0.f;         /**< @brief Y coordinate (m). */
    float z = 0.f;         /**< @brief Z coordinate (m). */
    float intensity = 0.f; /**< @brief Reflectivity. */
    float normal_x = 0.f;  /**< @brief Surface normal X. */
    float normal_y = 0.f;  /**< @brief Surface normal Y. */
    float normal_z = 0.f;  /**< @brief Surface normal Z. */
    float curvature =
        0.f; /**< @brief Relative scan time (ms) in LIVO pipelines. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief Colored point (BGRA channel order in memory). */
struct EIGEN_ALIGN16 PointXYZRGB {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    std::uint8_t b = 0;
    std::uint8_t g = 0;
    std::uint8_t r = 0;
    std::uint8_t a = 255;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief Colored point with explicit alpha (BGRA). */
struct EIGEN_ALIGN16 PointXYZRGBA {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    std::uint8_t b = 0;
    std::uint8_t g = 0;
    std::uint8_t r = 0;
    std::uint8_t a = 255;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief PCL-free point cloud container with optional organized layout
 * metadata.
 * @tparam PointT Point struct type (e.g. @ref PointXYZINormal).
 */
template <typename PointT>
class PointCloud
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(PointCloud<PointT>);
    /** @brief PCL-style aliases (same as SharedPtr / ConstSharedPtr). */
    using Ptr = SharedPtr;
    using ConstPtr = ConstSharedPtr;

    using PointType = PointT;
    using VectorType = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

    VectorType points; /**< @brief Stored points (aligned allocator). */
    std::uint32_t width =
        0; /**< @brief Organized width (unorganized: equals size). */
    std::uint32_t height =
        0; /**< @brief Organized height (1 for unorganized). */
    bool is_dense =
        true; /**< @brief False if NaNs or invalid points may exist. */

    PointCloud() = default;

    /**
     * @brief Preallocate @p n points in an unorganized cloud (height = 1).
     * @param n Number of points.
     * @param cols Unused (PCL compatibility).
     */
    PointCloud(std::size_t n, std::uint32_t /*cols*/)
        : points(n), width(static_cast<std::uint32_t>(n)), height(1) {}

    /** @return Number of points. */
    std::size_t size() const {
        return points.size();
    }
    /** @return True if @c points is empty. */
    bool empty() const {
        return points.empty();
    }

    /** @brief Clear points and reset width/height. */
    void clear() {
        points.clear();
        width = 0;
        height = 0;
    }

    /** @param n Capacity to reserve. */
    void reserve(std::size_t n) {
        points.reserve(n);
    }

    /**
     * @brief Resize point storage and update unorganized width/height.
     * @param n New point count.
     */
    void resize(std::size_t n) {
        points.resize(n);
        width = static_cast<std::uint32_t>(n);
        height = n > 0 ? 1 : 0;
    }

    /** @param p Point appended to the cloud. */
    void push_back(const PointT& p) {
        points.push_back(p);
        width = static_cast<std::uint32_t>(points.size());
        height = 1;
    }

    PointT& operator[](std::size_t i) {
        return points[i];
    }
    const PointT& operator[](std::size_t i) const {
        return points[i];
    }

    PointCloud& operator+=(const PointCloud& rhs) {
        points.insert(points.end(), rhs.points.begin(), rhs.points.end());
        width = static_cast<std::uint32_t>(points.size());
        height = points.empty() ? 0 : 1;
        return *this;
    }

    void swap(PointCloud& other) noexcept {
        points.swap(other.points);
        std::swap(width, other.width);
        std::swap(height, other.height);
        std::swap(is_dense, other.is_dense);
    }
};

using PointType = PointXYZINormal;
using PointTypeRGB = PointXYZRGB;
using PointTypeRGBA = PointXYZRGBA;
using PointCloudXYZI = PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using PointCloudXYZRGB = PointCloud<PointTypeRGB>;
using PointCloudXYZRGBA = PointCloud<PointTypeRGBA>;

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_TYPES_HPP_
