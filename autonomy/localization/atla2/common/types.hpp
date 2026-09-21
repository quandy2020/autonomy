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

#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace autonomy::localization::atla2 {

using Scalar = double;
using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
using Mat33 = Eigen::Matrix<Scalar, 3, 3>;
using Mat66 = Eigen::Matrix<Scalar, 6, 6>;
using MatXX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using Quat = Eigen::Quaternion<Scalar>;

//! Rigid pose (world ← body). Prefer Eigen to keep Atla2 free of SO3 helpers.
using SE3 = Eigen::Isometry3d;

inline SE3 Se3Identity() { return SE3::Identity(); }

inline Vec3 Se3Translation(const SE3& T) { return T.translation(); }

inline Quat Se3Rotation(const SE3& T) { return Quat(T.rotation()); }

inline Mat33 Se3RotationMatrix(const SE3& T) { return T.rotation(); }

inline SE3 MakeSe3(const Vec3& t, const Quat& q) {
  SE3 T = SE3::Identity();
  T.linear() = q.normalized().toRotationMatrix();
  T.translation() = t;
  return T;
}

inline SE3 Se3Inverse(const SE3& T) { return T.inverse(); }

//! Nanoseconds since Unix epoch (or a synchronized TimeBase).
using TimeStamp = int64_t;

inline constexpr TimeStamp kInvalidTime = -1;

enum class FrontendMode {
  kVo = 0,   // vision only
  kVio,      // vision + IMU
  kLo,       // lidar odometry (no IMU)
  kLio,      // lidar + IMU
  kLivo,     // lidar + vision + IMU
};

enum class BackendType {
  kIekf = 0,
  kGraph,   // alias for Ceres sliding window
  kCeres,   // Ceres VIO graph
};

enum class FusionStyle {
  kLoose = 0,
  kTight,
};

enum class SlamState {
  kUninitialized = 0,
  kInitializing,
  kTracking,
  kLost,
  kDegraded,
};

inline std::string ToString(FrontendMode m) {
  switch (m) {
    case FrontendMode::kVo:
      return "vo";
    case FrontendMode::kVio:
      return "vio";
    case FrontendMode::kLo:
      return "lo";
    case FrontendMode::kLio:
      return "lio";
    case FrontendMode::kLivo:
      return "livo";
  }
  return "unknown";
}

inline FrontendMode ParseFrontendMode(const std::string& s) {
  if (s == "vo") {
    return FrontendMode::kVo;
  }
  if (s == "lo") {
    return FrontendMode::kLo;
  }
  if (s == "lio") {
    return FrontendMode::kLio;
  }
  if (s == "livo") {
    return FrontendMode::kLivo;
  }
  return FrontendMode::kVio;
}

struct Landmark {
  int id = -1;
  Vec3 position = Vec3::Zero();
  Vec2 uv = Vec2::Zero();
  bool has_uv = false;
  Scalar inv_depth = 0.0;
};

struct PointXYZI {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
  double timestamp = 0.0;
};

using PointCloud = std::vector<PointXYZI>;

struct Covariance6 {
  Mat66 matrix = Mat66::Identity();
};

}  // namespace autonomy::localization::atla2
