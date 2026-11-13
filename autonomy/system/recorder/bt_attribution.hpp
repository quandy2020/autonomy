/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file bt_attribution.hpp
 * @brief L3: attribute a BT failure to the deepest FAILURE node in the episode.
 */

#include <optional>
#include <vector>

#include "autonomy/system/recorder/episode.hpp"
#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Attribute the episode failure to the last (deepest) FAILURE transition.
 * @param [in] transitions: BT transitions in the episode.
 * @return BtFailure proto; nullopt if no FAILURE appears.
 */
std::optional<proto::BtFailure> AttributeBtFailure(
    const std::vector<BtTransition>& transitions);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
