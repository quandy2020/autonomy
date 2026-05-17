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

#ifndef AUTONOMY_COMMON_NETWORK_NETWORK_HPP_
#define AUTONOMY_COMMON_NETWORK_NETWORK_HPP_

#include "autonomy/common/network/engine.hpp"
#include "autonomy/common/network/process.hpp"
#include "autonomy/common/network/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file network.hpp
 * @brief Umbrella header for model inference and generic pre/postprocess
 *
 * Includes @ref Engine, @ref TensorMap / @ref ModelTensorInfo, and the full
 * process API (@ref RunPipeline, @ref Preprocess, @ref Find, @ref Decode, etc.).
 * See README.md in this directory for usage examples.
 */

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_NETWORK_HPP_
