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

#include <deque>
#include <unordered_map>
#include <vector>

#include "autonomy/localization/atla2/backend/factors/pose_param.hpp"
#include "autonomy/localization/atla2/common/types.hpp"
#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

struct WindowFrame {
  TimeStamp t = kInvalidTime;
  double p[3] = {0, 0, 0};
  double aa[3] = {0, 0, 0};
  double v[3] = {0, 0, 0};
  //! Frontend prior pose (not modified by Ceres); used for relative measurements.
  double prior_p[3] = {0, 0, 0};
  double prior_aa[3] = {0, 0, 0};
  //! Preintegration from previous frame → this frame (body of prev).
  double dP[3] = {0, 0, 0};
  double dV[3] = {0, 0, 0};
  double dR_aa[3] = {0, 0, 0};
  double dt = 0.0;
  bool has_imu = false;
  //! Measured relative pose from frontend priors (prev → this).
  double meas_dp[3] = {0, 0, 0};
  double meas_daa[3] = {0, 0, 0};
  bool has_rel = false;
  std::vector<std::pair<int, Vec2>> observations;
};

struct WindowLandmark {
  int id = -1;
  double xyz[3] = {0, 0, 0};
  bool optimized = false;
};

class SlidingWindow {
 public:
  explicit SlidingWindow(int max_size = 10) : max_size_(max_size) {}

  void SetMaxSize(int n) { max_size_ = n > 2 ? n : 2; }
  int MaxSize() const { return max_size_; }
  void Clear();

  void Push(WindowFrame frame);
  std::deque<WindowFrame>& frames() { return frames_; }
  const std::deque<WindowFrame>& frames() const { return frames_; }

  WindowLandmark* GetOrCreateLandmark(int id, const Vec3& init_world);
  std::unordered_map<int, WindowLandmark>& landmarks() { return landmarks_; }

  double* ba() { return ba_; }
  double* bg() { return bg_; }

 private:
  int max_size_ = 10;
  std::deque<WindowFrame> frames_;
  std::unordered_map<int, WindowLandmark> landmarks_;
  double ba_[3] = {0, 0, 0};
  double bg_[3] = {0, 0, 0};
};

}  // namespace autonomy::localization::atla2
