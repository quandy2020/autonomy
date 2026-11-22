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
 * @file trace_context.hpp
 * @brief Thread-local trace / span ids.
 */

#include <string>

#include "common/proto/trace_metadata.pb.h"

namespace autoloop {
namespace robot {

void SetTrace(const common::TraceMetadata& meta);
common::TraceMetadata CurrentTrace();
std::string CurrentTraceId();

}  // namespace robot
}  // namespace autoloop
