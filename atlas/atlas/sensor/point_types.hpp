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
 * @brief PCL-free LiDAR point structs and cloud container (no protobuf).
 *
 * Kept separate from @ref point_cloud_data.hpp so algorithm code can include
 * point containers without pulling in automsgs / protobuf.
 */

#ifndef ATLAS_SENSOR_POINT_TYPES_HPP_
#define ATLAS_SENSOR_POINT_TYPES_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "common/macros.hpp"

namespace atlas {
namespace sensor {

/**
 * @brief Minimal LiDAR point: XYZ plus reflectivity.
 * @var x         X coordinate (metres).
 * @var y         Y coordinate (metres).
 * @var z         Z coordinate (metres).
 * @var intensity Reflectivity or normalized intensity.
 */
struct EIGEN_ALIGN16 PointXYZI {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief LIVO point: XYZ, intensity, optional normal, relative time in
 *        @c curvature.
 * @var x         X coordinate (metres).
 * @var y         Y coordinate (metres).
 * @var z         Z coordinate (metres).
 * @var intensity Reflectivity.
 * @var normal_x  Surface normal X (unitless).
 * @var normal_y  Surface normal Y (unitless).
 * @var normal_z  Surface normal Z (unitless).
 * @var curvature Relative scan time (ms) in LIVO pipelines.
 */
struct EIGEN_ALIGN16 PointXYZINormal {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
  float normal_x = 0.f;
  float normal_y = 0.f;
  float normal_z = 0.f;
  float curvature = 0.f;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Colored point (BGRA channel order in memory).
 * @var x X coordinate (metres).
 * @var y Y coordinate (metres).
 * @var z Z coordinate (metres).
 * @var b Blue channel 0–255.
 * @var g Green channel 0–255.
 * @var r Red channel 0–255.
 * @var a Alpha channel 0–255 (default 255).
 */
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

/**
 * @brief Colored point with explicit alpha (BGRA layout).
 * @var x X coordinate (metres).
 * @var y Y coordinate (metres).
 * @var z Z coordinate (metres).
 * @var b Blue channel 0–255.
 * @var g Green channel 0–255.
 * @var r Red channel 0–255.
 * @var a Alpha channel 0–255 (default 255).
 */
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
 *        metadata.
 * @tparam PointT Point struct type (e.g. @ref PointXYZINormal).
 * @var points   Stored points (Eigen-aligned allocator).
 * @var width    Organized width; for unorganized clouds equals @c size().
 * @var height   Organized height; @c 1 for unorganized clouds.
 * @var is_dense False if NaNs or invalid points may exist.
 */
template <typename PointT>
class LidarCloud {
 public:
  ATLAS_SHARED_PTR_DEFINITIONS(LidarCloud<PointT>);
  /** @brief PCL-style aliases (same as SharedPtr / ConstSharedPtr). */
  using Ptr = SharedPtr;
  using ConstPtr = ConstSharedPtr;

  using PointType = PointT;
  using VectorType = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

  VectorType points;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  bool is_dense = true;

  LidarCloud() = default;

  /**
   * @brief Preallocate @p n points in an unorganized cloud (@c height = 1).
   * @param[in] n    Number of points.
   * @param[in] cols Unused; retained for PCL compatibility.
   */
  LidarCloud(std::size_t n, std::uint32_t /*cols*/)
      : points(n), width(static_cast<std::uint32_t>(n)), height(1) {}

  /**
   * @brief Number of stored points.
   * @return @c points.size().
   */
  std::size_t size() const { return points.size(); }

  /**
   * @brief Whether the cloud has no points.
   * @return True when @c points is empty.
   */
  bool empty() const { return points.empty(); }

  /**
   * @brief Clear points and reset @c width / @c height to zero.
   */
  void clear() {
    points.clear();
    width = 0;
    height = 0;
  }

  /**
   * @brief Reserve storage capacity without changing size.
   * @param[in] n Capacity to reserve.
   */
  void reserve(std::size_t n) { points.reserve(n); }

  /**
   * @brief Resize point storage and update unorganized width/height.
   * @param[in] n New point count; @c height becomes 1 when @p n > 0.
   */
  void resize(std::size_t n) {
    points.resize(n);
    width = static_cast<std::uint32_t>(n);
    height = n > 0 ? 1 : 0;
  }

  /**
   * @brief Append one point and mark the cloud as unorganized.
   * @param[in] p Point copied onto the end of @c points.
   */
  void push_back(const PointT& p) {
    points.push_back(p);
    width = static_cast<std::uint32_t>(points.size());
    height = 1;
  }

  /**
   * @brief Mutable access by index (no bounds check).
   * @param[in] i Zero-based point index.
   * @return Reference to @c points[i].
   */
  PointT& operator[](std::size_t i) { return points[i]; }

  /**
   * @brief Const access by index (no bounds check).
   * @param[in] i Zero-based point index.
   * @return Const reference to @c points[i].
   */
  const PointT& operator[](std::size_t i) const { return points[i]; }

  /**
   * @brief Append all points from @p rhs; result is unorganized.
   * @param[in] rhs Cloud whose points are inserted at the end.
   * @return Reference to this cloud.
   */
  LidarCloud& operator+=(const LidarCloud& rhs) {
    points.insert(points.end(), rhs.points.begin(), rhs.points.end());
    width = static_cast<std::uint32_t>(points.size());
    height = points.empty() ? 0 : 1;
    return *this;
  }

  /**
   * @brief Swap contents with @p other (points, layout, density flag).
   * @param[in,out] other Other cloud; exchanged with this.
   */
  void swap(LidarCloud& other) noexcept {
    points.swap(other.points);
    std::swap(width, other.width);
    std::swap(height, other.height);
    std::swap(is_dense, other.is_dense);
  }
};

using PointType = PointXYZINormal;
using PointTypeRGB = PointXYZRGB;
using PointTypeRGBA = PointXYZRGBA;
using PointCloudXYZI = LidarCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using PointCloudXYZRGB = LidarCloud<PointTypeRGB>;
using PointCloudXYZRGBA = LidarCloud<PointTypeRGBA>;

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_POINT_TYPES_HPP_
