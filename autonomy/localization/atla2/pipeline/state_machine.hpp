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

#include "autonomy/localization/atla2/common/types.hpp"

namespace autonomy::localization::atla2 {

class StateMachine {
 public:
  SlamState state() const { return state_; }

  void OnInitSuccess() { state_ = SlamState::kTracking; }
  void OnInitStart() { state_ = SlamState::kInitializing; }
  void OnTrackingOk() { state_ = SlamState::kTracking; }
  void OnLost() { state_ = SlamState::kLost; }
  void OnDegraded() { state_ = SlamState::kDegraded; }
  void Reset() { state_ = SlamState::kUninitialized; }

 private:
  SlamState state_ = SlamState::kUninitialized;
};

}  // namespace autonomy::localization::atla2
