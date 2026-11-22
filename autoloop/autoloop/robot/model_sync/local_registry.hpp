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
 * @file local_registry.hpp
 * @brief On-disk model pointer (atomic switch of current -> next).
 */
#include <string>
#include "common/base/status.hpp"

namespace autoloop {
namespace robot {

class LocalRegistry {
public:
    explicit LocalRegistry(std::string root);
    common::Status SwitchTo(const std::string& version);
    std::string Current() const;

private:
    std::string root_;
};

}  // namespace robot
}  // namespace autoloop
