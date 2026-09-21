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

#include "autonomy/localization/atla2/backend/graph/sliding_window.hpp"

namespace autonomy::localization::atla2 {

void SlidingWindow::Clear() {
  frames_.clear();
  landmarks_.clear();
  ba_[0] = ba_[1] = ba_[2] = 0;
  bg_[0] = bg_[1] = bg_[2] = 0;
}

void SlidingWindow::Push(WindowFrame frame) {
  frames_.push_back(std::move(frame));
  while (static_cast<int>(frames_.size()) > max_size_) {
    frames_.pop_front();
  }
}

WindowLandmark* SlidingWindow::GetOrCreateLandmark(int id, const Vec3& init_world) {
  auto it = landmarks_.find(id);
  if (it != landmarks_.end()) {
    return &it->second;
  }
  WindowLandmark lm;
  lm.id = id;
  Vec3ToArray(init_world, lm.xyz);
  auto [ins, _] = landmarks_.emplace(id, lm);
  return &ins->second;
}

}  // namespace autonomy::localization::atla2
