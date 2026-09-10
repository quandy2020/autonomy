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

/**
 * @file options.hpp
 * @brief Validation and lookup helpers for YOLO26 base perception config.
 */

#ifndef AUTONOMY_PERCEPTION_BASE_OPTIONS_HPP_
#define AUTONOMY_PERCEPTION_BASE_OPTIONS_HPP_

#include "autonomy/perception/base/proto/base.pb.h"

#include <string>

namespace autonomy {
namespace perception {
namespace base {

/** @brief Validates backend, input size, and each enabled base task. */
bool ValidateBaseOptions(const proto::BaseOptions& options,
                         std::string* error = nullptr);

/** @brief Maps Backend enum to common-network backend_id string. */
std::string BackendId(proto::Backend backend);

/** @brief First TaskOptions with matching kind, or nullptr. */
const proto::TaskOptions* FindTask(const proto::BaseOptions& options,
                                   proto::TaskKind kind);

/** @brief True when a TaskOptions entry exists with enable=true. */
bool TaskEnabled(const proto::BaseOptions& options, proto::TaskKind kind);

}  // namespace base
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_BASE_OPTIONS_HPP_
