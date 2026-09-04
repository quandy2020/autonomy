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

#ifndef AUTONOMY_PERCEPTION_FATHOM_ENGINE_MODEL_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_ENGINE_MODEL_HPP_

#include "autonomy/perception/fathom/depth/refiner.hpp"

#include <memory>
#include <string>

namespace autonomy {
namespace common {
namespace network {
class Engine;
}  // namespace network
}  // namespace common

namespace perception {
namespace fathom {

/**
 * @file model.hpp
 * @brief Fathom-specific runner and common-network adapter.
 */

/**
 * Adapts the fixed Fathom ONNX graph to common::network::Engine.
 *
 * This is deliberately the only concrete inference adapter in the Fathom
 * module. The depth facade depends on FathomModelRunner so it remains testable
 * with a small fake runner and has no backend type in its public API.
 */
class FathomEngine final : public FathomModelRunner {
public:
    static std::unique_ptr<FathomEngine> Create(const FathomConfig& config,
                                                std::string* error = nullptr);

    ~FathomEngine() override;

    FathomEngine(const FathomEngine&) = delete;
    FathomEngine& operator=(const FathomEngine&) = delete;
    FathomEngine(FathomEngine&&) = delete;
    FathomEngine& operator=(FathomEngine&&) = delete;

    bool Run(const common::network::TensorMap& inputs,
             common::network::TensorMap* outputs,
             std::string* error = nullptr) override;

private:
    explicit FathomEngine(std::unique_ptr<common::network::Engine> engine);

    std::unique_ptr<common::network::Engine> engine_;
};

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_ENGINE_MODEL_HPP_
