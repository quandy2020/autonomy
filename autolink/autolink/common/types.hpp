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

#include <cstdint>

namespace autolink {

class NullType
{
};

// Return code definition for autolink internal function return.
enum ReturnCode {
    SUCC = 0,
    FAIL = 1,
};

/**
 * @brief Describe relation between nodes, writers/readers...
 */
enum Relation : std::uint8_t {
    NO_RELATION = 0,
    DIFF_HOST,  // different host
    DIFF_PROC,  // same host, but different process
    SAME_PROC,  // same process
};

static const char SRV_CHANNEL_REQ_SUFFIX[] = "__SRV__REQUEST";
static const char SRV_CHANNEL_RES_SUFFIX[] = "__SRV__RESPONSE";

}  // namespace autolink
