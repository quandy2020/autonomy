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

//! Backend module umbrella (§2b).

#include "autonomy/localization/atlas/backend/global_joint_ba.hpp"
#include "autonomy/localization/atlas/backend/loop_closing.hpp"

namespace autonomy::localization::atlas {
namespace backend {

using LoopClosing = ::autonomy::localization::atlas::LoopClosing;
using global_optimization_module = LoopClosing;
// GlobalJointBA is the canonical class in this namespace.

struct BackendModuleTag {};

}  // namespace backend
}  // namespace autonomy::localization::atlas
