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

#include "autonomy/perception/fathom/model.hpp"

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

bool MatchesShape(const common::network::TensorShape& shape,
                  const std::vector<int64_t>& expected) {
  return shape.Rank() == expected.size() && shape.Dims() == expected;
}

bool ValidateTensor(const common::network::ModelTensorInfo& info,
                    const char* label, const std::vector<int64_t>& expected,
                    std::string* error) {
  if (info.element_type != common::network::ElementType::kFloat32) {
    SetError(error, std::string(label) + " must use float32 elements.");
    return false;
  }
  if (!MatchesShape(info.shape, expected)) {
    SetError(error, std::string(label) + " has an unexpected static shape.");
    return false;
  }
  return true;
}

bool ValidateRawDepthShape(const common::network::ModelTensorInfo& info,
                           int64_t height, int64_t width, std::string* error) {
  // Fathom ONNX export: [1, H, W]. lingbot_depth_trt engines: [1, 1, H, W].
  if (ValidateTensor(info, "model input 'raw_depth'", {1, height, width},
                     error)) {
    return true;
  }
  if (error != nullptr) {
    error->clear();
  }
  return ValidateTensor(info, "model input 'raw_depth'", {1, 1, height, width},
                        error);
}

bool ValidateModelContract(const common::network::Engine& engine,
                           const proto::FathomOptions& options,
                           std::string* error) {
  const auto inputs = engine.GetInputInfos();
  const auto outputs = engine.GetOutputInfos();
  const auto* image = FindTensorInfo(inputs, "image");
  const auto* raw_depth = FindTensorInfo(inputs, "raw_depth");
  if (inputs.size() != 2 || image == nullptr || raw_depth == nullptr) {
    SetError(error,
             "model inputs must be exactly 'image' and 'raw_depth'.");
    return false;
  }

  const auto* refined_depth = FindTensorInfo(outputs, "refined_depth");
  const auto* pred_depth = FindTensorInfo(outputs, "pred_depth");
  const auto* validity = FindTensorInfo(outputs, "validity");
  const common::network::ModelTensorInfo* depth_out = refined_depth;
  if (depth_out == nullptr) {
    depth_out = pred_depth;
  }
  if (depth_out == nullptr) {
    SetError(error,
             "model outputs must include 'refined_depth' or 'pred_depth'.");
    return false;
  }
  // Accept Fathom dual-output graphs or lingbot single-output TRT engines.
  if (outputs.size() == 2) {
    if (validity == nullptr) {
      SetError(error,
               "two-output models must provide 'validity' with the depth map.");
      return false;
    }
  } else if (outputs.size() != 1) {
    SetError(error,
             "model must expose one depth output, or depth+validity.");
    return false;
  }

  const int64_t width = static_cast<int64_t>(options.input_width());
  const int64_t height = static_cast<int64_t>(options.input_height());
  if (!ValidateTensor(*image, "model input 'image'", {1, 3, height, width},
                      error) ||
      !ValidateRawDepthShape(*raw_depth, height, width, error) ||
      !ValidateTensor(*depth_out, "model depth output", {1, height, width},
                      error)) {
    return false;
  }
  if (validity != nullptr &&
      !ValidateTensor(*validity, "model output 'validity'", {1, height, width},
                      error)) {
    return false;
  }
  return true;
}

}  // namespace

FathomEngine::FathomEngine(std::unique_ptr<common::network::Engine> engine)
    : engine_(std::move(engine)) {}

FathomEngine::~FathomEngine() = default;

std::unique_ptr<FathomEngine> FathomEngine::Create(
    const proto::FathomOptions& options, std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (!ValidateModelOptions(options, error)) {
    return nullptr;
  }

  common::network::InferenceOptions inference_options;
  inference_options.backend_id = options.backend();
  inference_options.model_path = options.model_path();
  std::string engine_error;
  auto engine =
      common::network::Engine::CreateEngine(inference_options, &engine_error);
  if (engine == nullptr) {
    SetError(error, engine_error);
    return nullptr;
  }
  if (!ValidateModelContract(*engine, options, error)) {
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
