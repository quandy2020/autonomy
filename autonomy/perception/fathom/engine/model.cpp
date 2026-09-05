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

/**
 * @file model.cpp
 * @brief Fathom model loading, tensor-contract validation, and inference.
 */

#include "autonomy/perception/fathom/engine/model.hpp"

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/common/network/common/options.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace fathom {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Fathom: " + message;
    }
}

const common::network::ModelTensorInfo* FindTensorInfo(
    const std::vector<common::network::ModelTensorInfo>& infos,
    const char* name) {
    for (const auto& info : infos) {
        if (info.name == name) {
            return &info;
        }
    }
    return nullptr;
}

bool ValidateTensor(const common::network::ModelTensorInfo& info,
                    const char* label, const std::vector<int64_t>& expected,
                    std::string* error) {
    if (info.element_type != common::network::ElementType::kFloat32) {
        SetError(error, std::string(label) + " must use float32 elements.");
        return false;
    }
    if (info.shape.Rank() != expected.size() || info.shape.Dims() != expected) {
        SetError(error,
                 std::string(label) + " has an unexpected static shape.");
        return false;
    }
    return true;
}

bool ValidateModelContract(const common::network::Engine& engine,
                           const FathomConfig& config, std::string* error) {
    const auto inputs = engine.GetInputInfos();
    const auto outputs = engine.GetOutputInfos();
    const auto* image = FindTensorInfo(inputs, "image");
    const auto* raw_depth = FindTensorInfo(inputs, "raw_depth");
    const auto* refined_depth = FindTensorInfo(outputs, "refined_depth");
    const auto* validity = FindTensorInfo(outputs, "validity");
    if (inputs.size() != 2 || image == nullptr || raw_depth == nullptr) {
        SetError(error,
                 "model inputs must be exactly 'image' and 'raw_depth'.");
        return false;
    }
    if (outputs.size() != 2 || refined_depth == nullptr ||
        validity == nullptr) {
        SetError(
            error,
            "model outputs must be exactly 'refined_depth' and 'validity'.");
        return false;
    }

    // common::network::Engine exposes the graph's tensor metadata, not the
    // Python token-count constant baked into the exported graph. Validate the
    // observable fixed spatial contract here; token selection remains an
    // export-time artifact property.
    const int64_t width = static_cast<int64_t>(config.input_width);
    const int64_t height = static_cast<int64_t>(config.input_height);
    return ValidateTensor(*image, "model input 'image'", {1, 3, height, width},
                          error) &&
           ValidateTensor(*raw_depth, "model input 'raw_depth'",
                          {1, height, width}, error) &&
           ValidateTensor(*refined_depth, "model output 'refined_depth'",
                          {1, height, width}, error) &&
           ValidateTensor(*validity, "model output 'validity'",
                          {1, height, width}, error);
}

}  // namespace

FathomEngine::FathomEngine(std::unique_ptr<common::network::Engine> engine)
    : engine_(std::move(engine)) {}

FathomEngine::~FathomEngine() = default;

std::unique_ptr<FathomEngine> FathomEngine::Create(const FathomConfig& config,
                                                   std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!ValidateFathomConfig(config, error)) {
        return nullptr;
    }

    common::network::InferenceOptions options;
    options.backend_id = config.backend;
    options.model_path = config.model_path;
    std::string engine_error;
    auto engine = common::network::Engine::CreateEngine(options, &engine_error);
    if (engine == nullptr) {
        SetError(error, engine_error);
        return nullptr;
    }
    if (!ValidateModelContract(*engine, config, error)) {
        return nullptr;
    }
    return std::unique_ptr<FathomEngine>(new FathomEngine(std::move(engine)));
}

bool FathomEngine::Run(const common::network::TensorMap& inputs,
                       common::network::TensorMap* outputs,
                       std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (outputs == nullptr) {
        SetError(error, "model output map is null.");
        return false;
    }
    outputs->clear();
    if (!engine_->Run(inputs, outputs)) {
        SetError(error, engine_->GetLastError());
        outputs->clear();
        return false;
    }
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
