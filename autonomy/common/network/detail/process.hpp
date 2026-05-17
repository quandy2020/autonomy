/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_COMMON_NETWORK_DETAIL_PROCESS_HPP_
#define AUTONOMY_COMMON_NETWORK_DETAIL_PROCESS_HPP_

#include "autonomy/common/network/detail/postprocess/boxes.hpp"
#include "autonomy/common/network/detail/postprocess/cls.hpp"
#include "autonomy/common/network/detail/postprocess/map.hpp"
#include "autonomy/common/network/detail/postprocess/nms.hpp"
#include "autonomy/common/network/detail/postprocess/types.hpp"
#include "autonomy/common/network/detail/preprocess/dims.hpp"
#include "autonomy/common/network/detail/preprocess/image.hpp"
#include "autonomy/common/network/detail/preprocess/inputs.hpp"
#include "autonomy/common/network/detail/preprocess/layout.hpp"
#include "autonomy/common/network/detail/preprocess/norm.hpp"
#include "autonomy/common/network/detail/preprocess/policy.hpp"
#include "autonomy/common/network/detail/preprocess/resize.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file process.hpp
 * @brief Internal umbrella: all preprocess and postprocess module headers
 *
 * Application code should include autonomy/common/network/process.hpp instead.
 */

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_DETAIL_PROCESS_HPP_
