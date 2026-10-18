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

#include <cmath>
#include <iostream>

#include "argus/common/math/se3.hpp"
#include "argus/common/math/transform/timestamped_transform.hpp"
#include "argus/common/math/transform/transform.hpp"
#include "argus/common/math/transform/transform_interpolation_buffer.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

}  // namespace

int main() {
  using argus::math::SE3;
  using argus::transform::Embed3D;
  using argus::transform::FromXyzYaw;
  using argus::transform::GetAngle;
  using argus::transform::GetYaw;
  using argus::transform::Interpolate;
  using argus::transform::Project2D;
  using argus::transform::Rigid2d;
  using argus::transform::Rigid3d;
  using argus::transform::RollPitchYaw;
  using argus::transform::TimestampedTransform;
  using argus::transform::ToIsometry;
  using argus::transform::ToRigid3;
  using argus::transform::TransformInterpolationBuffer;

  const Rigid3d T = FromXyzYaw(1.0, 2.0, 3.0, 0.5);
  if ((T.translation() - Eigen::Vector3d(1, 2, 3)).norm() > 1e-9) {
    return Fail("FromXyzYaw translation");
  }
  if (std::abs(GetYaw(T) - 0.5) > 1e-9) {
    return Fail("GetYaw");
  }

  const Rigid3d inv = T.inverse();
  const Eigen::Vector3d p_b(0.1, -0.2, 0.3);
  const Eigen::Vector3d p_w = T * p_b;
  if ((inv * p_w - p_b).norm() > 1e-9) {
    return Fail("Rigid3 inverse compose");
  }

  const Rigid2d T2 = Project2D(T);
  const Rigid3d T3 = Embed3D(T2);
  if (std::abs(T3.translation().z()) > 1e-12) {
    return Fail("Embed3D z");
  }
  if (std::abs(GetYaw(T3) - 0.5) > 1e-9) {
    return Fail("Embed3D yaw");
  }

  const Eigen::Quaterniond q = RollPitchYaw(0.1, -0.2, 0.3);
  if (std::abs(q.norm() - 1.0) > 1e-9) {
    return Fail("RollPitchYaw unit");
  }

  const SE3 se3(T);
  if ((se3.translation() - T.translation()).norm() > 1e-12) {
    return Fail("SE3 facade");
  }
  if ((ToIsometry(T).translation() - T.translation()).norm() > 1e-12) {
    return Fail("ToIsometry");
  }
  if ((ToRigid3(ToIsometry(T)).translation() - T.translation()).norm() >
      1e-12) {
    return Fail("ToRigid3 roundtrip");
  }

  TimestampedTransform a{0.0, Rigid3d::Identity()};
  TimestampedTransform b{1.0, FromXyzYaw(2.0, 0.0, 0.0, 0.0)};
  const auto mid = Interpolate(a, b, 0.5);
  if (std::abs(mid.transform.translation().x() - 1.0) > 1e-9) {
    return Fail("Interpolate translation");
  }
  if (GetAngle(mid.transform) > 1e-9) {
    return Fail("Interpolate rotation identity");
  }

  TransformInterpolationBuffer buf;
  buf.Push(0.0, Rigid3d::Identity());
  buf.Push(1.0, FromXyzYaw(2.0, 0.0, 0.0, 0.0));
  if (!buf.Has(0.5)) {
    return Fail("buffer Has");
  }
  const Rigid3d Tl = buf.Lookup(0.5);
  if (std::abs(Tl.translation().x() - 1.0) > 1e-9) {
    return Fail("buffer Lookup");
  }

  std::cout << "OK transform\n";
  return 0;
}
