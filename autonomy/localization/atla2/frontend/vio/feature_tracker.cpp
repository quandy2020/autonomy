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

#include "autonomy/localization/atla2/frontend/vio/feature_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy::localization::atla2 {

namespace {

inline uint8_t At(const ImageFrame& img, int x, int y) {
  if (x < 0 || y < 0 || x >= img.width || y >= img.height || img.data.empty()) {
    return 0;
  }
  const int stride = img.width * std::max(1, img.channels);
  return img.data[static_cast<size_t>(y * stride + x * img.channels)];
}

float CornerScore(const ImageFrame& img, int x, int y) {
  // Simple Harris-like: Ix^2 * Iy^2 - (IxIy)^2
  const float dx = static_cast<float>(At(img, x + 1, y)) - static_cast<float>(At(img, x - 1, y));
  const float dy = static_cast<float>(At(img, x, y + 1)) - static_cast<float>(At(img, x, y - 1));
  return dx * dx + dy * dy;
}

float PatchSad(const ImageFrame& a, int ax, int ay, const ImageFrame& b, int bx, int by,
               int r) {
  float sad = 0.f;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      sad += std::fabs(static_cast<float>(At(a, ax + dx, ay + dy)) -
                       static_cast<float>(At(b, bx + dx, by + dy)));
    }
  }
  return sad;
}

}  // namespace

void FeatureTracker::Reset() {
  feats_.clear();
  prev_ = ImageFrame{};
  next_id_ = 0;
}

void FeatureTracker::Detect(const ImageFrame& image) {
  if (image.width <= 0 || image.height <= 0 || image.data.empty()) {
    return;
  }
  const int cell_w = std::max(1, image.width / opt_.grid_cols);
  const int cell_h = std::max(1, image.height / opt_.grid_rows);
  const int margin = opt_.patch_radius + 1;

  std::vector<bool> occupied(static_cast<size_t>(opt_.grid_rows * opt_.grid_cols), false);
  for (const auto& f : feats_) {
    if (!f.tracked) {
      continue;
    }
    const int cx = static_cast<int>(f.uv.x()) / cell_w;
    const int cy = static_cast<int>(f.uv.y()) / cell_h;
    if (cx >= 0 && cy >= 0 && cx < opt_.grid_cols && cy < opt_.grid_rows) {
      occupied[static_cast<size_t>(cy * opt_.grid_cols + cx)] = true;
    }
  }

  for (int gy = 0; gy < opt_.grid_rows; ++gy) {
    for (int gx = 0; gx < opt_.grid_cols; ++gx) {
      if (occupied[static_cast<size_t>(gy * opt_.grid_cols + gx)]) {
        continue;
      }
      if (static_cast<int>(feats_.size()) >= opt_.max_features) {
        return;
      }
      const int x0 = std::max(margin, gx * cell_w);
      const int y0 = std::max(margin, gy * cell_h);
      const int x1 = std::min(image.width - margin, (gx + 1) * cell_w);
      const int y1 = std::min(image.height - margin, (gy + 1) * cell_h);
      float best = opt_.min_score;
      int bx = -1, by = -1;
      for (int y = y0; y < y1; y += 2) {
        for (int x = x0; x < x1; x += 2) {
          const float s = CornerScore(image, x, y);
          if (s > best) {
            best = s;
            bx = x;
            by = y;
          }
        }
      }
      if (bx >= 0) {
        TrackedFeature f;
        f.id = next_id_++;
        f.uv = Vec2(bx, by);
        f.uv_prev = f.uv;
        f.age = 0;
        f.tracked = true;
        feats_.push_back(f);
      }
    }
  }
}

void FeatureTracker::Track(const ImageFrame& prev, const ImageFrame& cur) {
  const int r = opt_.patch_radius;
  const int sr = opt_.search_radius;
  for (auto& f : feats_) {
    const int px = static_cast<int>(f.uv.x());
    const int py = static_cast<int>(f.uv.y());
    float best = std::numeric_limits<float>::max();
    int bx = px, by = py;
    for (int dy = -sr; dy <= sr; ++dy) {
      for (int dx = -sr; dx <= sr; ++dx) {
        const int nx = px + dx;
        const int ny = py + dy;
        if (nx < r || ny < r || nx >= cur.width - r || ny >= cur.height - r) {
          continue;
        }
        const float sad = PatchSad(prev, px, py, cur, nx, ny, r);
        if (sad < best) {
          best = sad;
          bx = nx;
          by = ny;
        }
      }
    }
    const float thresh = static_cast<float>((2 * r + 1) * (2 * r + 1) * 25);
    f.uv_prev = f.uv;
    if (best < thresh) {
      f.uv = Vec2(bx, by);
      f.tracked = true;
      ++f.age;
    } else {
      f.tracked = false;
    }
  }
  feats_.erase(std::remove_if(feats_.begin(), feats_.end(),
                              [](const TrackedFeature& f) { return !f.tracked; }),
               feats_.end());
}

std::vector<TrackedFeature> FeatureTracker::Process(const ImageFrame& image) {
  if (prev_.width > 0 && !prev_.data.empty() && !feats_.empty()) {
    Track(prev_, image);
  } else {
    feats_.clear();
  }
  Detect(image);
  prev_ = image;
  return feats_;
}

}  // namespace autonomy::localization::atla2
