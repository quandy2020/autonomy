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
 * @file runner.hpp
 * @brief Backend-independent inference contract for prepared tensors.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_RUNNER_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_RUNNER_HPP_

#include "autonomy/common/network/common/tensor.hpp"

#include <string>

namespace autonomy {
namespace perception {
namespace hestia {

class Runner
{
public:
    virtual ~Runner() = default;

    virtual bool Run(const common::network::TensorMap& inputs,
                     common::network::TensorMap* outputs,
                     std::string* error = nullptr) = 0;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_RUNNER_HPP_
