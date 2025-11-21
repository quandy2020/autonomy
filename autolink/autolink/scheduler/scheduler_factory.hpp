/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autolink/common/environment.hpp"
#include "autolink/common/file.hpp"
#include "autolink/common/global_data.hpp"
#include "autolink/common/util.hpp"
#include "autolink/scheduler/policy/scheduler_choreography.hpp"
#include "autolink/scheduler/policy/scheduler_classic.hpp"
#include "autolink/scheduler/scheduler.hpp"

namespace autolink {
namespace scheduler {

Scheduler* Instance();

void CleanUp();

}  // namespace scheduler
}  // namespace autolink