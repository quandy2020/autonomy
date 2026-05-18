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

#pragma once

/**
 * @file io.hpp
 * @brief ONNX Runtime tensor create/copy helpers (library-internal, not
 * installed)
 *
 * Used by @ref OnnxBackend::Run. Not a stable public ABI.
 */

#include <onnxruntime_cxx_api.h>

#include <string>

#include "autonomy/common/network/common/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {
namespace onnx {
namespace detail {

bool MapOrtElementType(ONNXTensorElementDataType ort_type, ElementType* out);

bool CreateOrtInputTensor(const ModelTensorInfo& meta, const Tensor& host,
                          Ort::MemoryInfo& memory_info, Ort::Value* out,
                          std::string* error);

bool CopyOrtOutputTensor(Ort::Value& ort_value, const ModelTensorInfo& meta,
                         Tensor* out, std::string* error);

/** @brief True when every output has a known static shape suitable for
 * IoBinding */
bool CanUseIoBinding(const std::vector<ModelTensorInfo>& output_infos);

/**
 * @brief Runs inference (IoBinding when @p prefer_io_binding and outputs are
 * static)
 *
 * Inputs borrow host pointers from @p inputs; outputs are filled in @p outputs.
 */
bool RunInference(Ort::Session& session,
                  const std::vector<ModelTensorInfo>& input_infos,
                  const std::vector<ModelTensorInfo>& output_infos,
                  const TensorMap& inputs, TensorMap* outputs,
                  bool prefer_io_binding, std::string* error);

}  // namespace detail
}  // namespace onnx
}  // namespace network
}  // namespace common
}  // namespace autonomy
