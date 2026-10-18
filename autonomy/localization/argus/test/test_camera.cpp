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
 * @file test_camera.cpp
 * @brief Round-trip smoke for rpg_vikit-compatible camera models.
 */

#include <cmath>
#include <iostream>

#include "argus/common/camera/atan_camera.hpp"
#include "argus/common/camera/camera_loader.hpp"
#include "argus/common/camera/equidistant_camera.hpp"
#include "argus/common/camera/omni_camera.hpp"
#include "argus/common/camera/pinhole_camera.hpp"
#include "argus/common/camera/polynomial_camera.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

bool BearingRoundTrip(const argus::camera::AbstractCamera& cam, double u,
                      double v, double tol_px = 0.5) {
  const Eigen::Vector3d b = cam.cam2world(u, v);
  const Eigen::Vector2d px = cam.world2cam(b);
  return std::abs(px[0] - u) < tol_px && std::abs(px[1] - v) < tol_px;
}

}  // namespace

int main() {
  using argus::camera::ATANCamera;
  using argus::camera::CameraIntrinsics;
  using argus::camera::Create;
  using argus::camera::EquidistantCamera;
  using argus::camera::OcamModel;
  using argus::camera::OmniCamera;
  using argus::camera::PinholeCamera;
  using argus::camera::PolynomialCamera;

  {
    PinholeCamera cam(320, 240, 1.0, 200, 200, 160, 120);
    if (!BearingRoundTrip(cam, 160, 120)) {
      return Fail("pinhole center");
    }
    if (!BearingRoundTrip(cam, 200, 100)) {
      return Fail("pinhole offset");
    }
  }

  {
    EquidistantCamera cam(320, 240, 1.0, 200, 200, 160, 120, 0.01, 0.0, 0.0,
                          0.0);
    if (!BearingRoundTrip(cam, 180, 130, 1.0)) {
      return Fail("equidistant");
    }
  }

  {
    ATANCamera cam(320, 240, 1.0, 200, 200, 160, 120, 0.5);
    if (!BearingRoundTrip(cam, 180, 130, 1.5)) {
      return Fail("atan");
    }
    auto n = ATANCamera::FromNormalized(320, 240, 200.0 / 320.0, 200.0 / 240.0,
                                        0.5, 0.5, 0.0);
    if (!BearingRoundTrip(n, 160, 120, 1.0)) {
      return Fail("atan normalized");
    }
  }

  {
    PolynomialCamera cam(320, 240, 1.0, 200, 200, 160, 120, 0.0, 0.01);
    if (!BearingRoundTrip(cam, 180, 130, 2.0)) {
      return Fail("polynomial");
    }
  }

  {
    OcamModel m;
    m.length_pol = 5;
    m.pol[0] = -2.0e2;
    m.pol[1] = 0.0;
    m.pol[2] = 2.0e-3;
    m.pol[3] = 0.0;
    m.pol[4] = 0.0;
    m.length_invpol = 5;
    m.invpol[0] = 200.0;
    m.invpol[1] = 0.0;
    m.invpol[2] = 0.0;
    m.invpol[3] = 0.0;
    m.invpol[4] = 0.0;
    m.xc = 120.0;
    m.yc = 160.0;
    m.c = 1.0;
    m.d = 0.0;
    m.e = 0.0;
    m.width = 320;
    m.height = 240;
    OmniCamera cam(m);
    if (!cam.valid()) {
      return Fail("omni valid");
    }
    const Eigen::Vector3d b = cam.cam2world(160, 120);
    if (b.norm() < 0.5) {
      return Fail("omni bearing");
    }
    const Eigen::Vector2d px = cam.world2cam(b);
    if (!std::isfinite(px[0]) || !std::isfinite(px[1])) {
      return Fail("omni world2cam");
    }
  }

  {
    CameraIntrinsics i;
    i.model = "FOV";
    i.width = 100;
    i.height = 80;
    i.scale = 1.0;
    i.fx = 50;
    i.fy = 50;
    i.cx = 50;
    i.cy = 40;
    i.s = 0.0;
    auto cam = Create(i);
    if (!cam) {
      return Fail("loader ATAN alias");
    }
  }

  std::cout << "OK camera (pinhole/equidistant/atan/polynomial/omni)\n";
  return 0;
}
