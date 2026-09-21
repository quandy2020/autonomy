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

#include <utility>
#include <vector>

#include "autonomy/localization/atla2/common/types.hpp"
#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

struct TrackedFeature {
  int id = -1;
  Vec2 uv = Vec2::Zero();
  Vec2 uv_prev = Vec2::Zero();
  int age = 0;
  bool tracked = false;
};

//! Lightweight intensity corner detector + patch SSD tracker (no OpenCV).
class FeatureTracker {
 public:
  struct Options {
    int max_features = 150;
    int grid_rows = 6;
    int grid_cols = 8;
    int patch_radius = 3;
    int search_radius = 8;
    float min_score = 30.f;
  };

  FeatureTracker() : opt_{} {}
  explicit FeatureTracker(Options opt) : opt_(std::move(opt)) {}

  void Reset();
  //! Detect / track on grayscale (or first channel of multi-channel).
  std::vector<TrackedFeature> Process(const ImageFrame& image);

  const std::vector<TrackedFeature>& Features() const { return feats_; }

 private:
  void Detect(const ImageFrame& image);
  void Track(const ImageFrame& prev, const ImageFrame& cur);

  Options opt_;
  int next_id_ = 0;
  ImageFrame prev_;
  std::vector<TrackedFeature> feats_;
};

}  // namespace autonomy::localization::atla2
