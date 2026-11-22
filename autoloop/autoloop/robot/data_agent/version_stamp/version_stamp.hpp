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
 * @file version_stamp.hpp
 * @brief Stamp sidecar / health with the version quadruple.
 */

#include "robot/data_agent/version_stamp/version_tuple.hpp"

namespace autoloop {
namespace robot {

class VersionStamp {
public:
    VersionStamp() : check_(MakeVersionTuple()) {}
    const VersionCheck& check() const { return check_; }

private:
    VersionCheck check_;
};

}  // namespace robot
}  // namespace autoloop
