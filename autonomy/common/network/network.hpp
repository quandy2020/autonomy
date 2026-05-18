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

/**
 * @file network.hpp
 * @brief Application umbrella header: inference, preprocess, postprocess,
 * pipeline
 *
 * @namespace autonomy::common::network
 *
 * ## Responsibilities
 *
 * - **Inference**: @ref Engine loads ONNX / TensorRT models and runs forward
 * passes
 * - **Preprocess**: @ref Preprocess converts BGR images, vectors, or pre-filled
 * tensors
 * - **Postprocess**: @ref FindFloatOutput, @ref Decode, @ref ToMat, etc.
 * - **Pipeline**: @ref RunPipeline chains preprocess and @ref Engine::Run
 *
 * ## Recommended includes
 *
 * Application code should include this header only. For custom @ref Backend
 * extensions, also include `backend/backend.hpp` and the concrete backend (e.g.
 * `backend/onnx/onnx.hpp`).
 *
 * ## Example
 *
 * @code
 * InferenceOptions opt;
 * opt.model_path = "/path/to/model.onnx";
 * std::string err;
 * auto engine = Engine::CreateEngine(opt, &err);
 *
 * PreprocessOptions prep = Letterbox<640, 640>();
 * RunResult result;
 * if (!RunPipeline(engine.get(), image, prep, &result, &err)) { ... }
 *
 * const std::vector<float>* depth = nullptr;
 * std::string out_name;
 * FindFloatOutput(result.outputs, engine->GetOutputInfos(),
 *                 &out_name, &depth, "depth", &err);
 * @endcode
 *
 * ## Runtime types
 *
 * Model I/O uses @ref Tensor / @ref TensorMap (float32, float16, bfloat16,
 * int8, …).
 * @ref Engine::Run(const FloatTensorMap&, …) is a float32 convenience overload.
 * Preprocess converts to each model input `element_type` via @ref
 * FromPreprocessFloat. Set `onnx.execution_provider` to `"cuda"` for GPU
 * inference when ORT is built with CUDA. Set `onnx.use_io_binding` to reduce
 * host output copies when output shapes are static.
 *
 * @note Do not include `detail/**` or `backend/onnx/io.hpp` from application
 * code.
 * @note @ref Engine is not thread-safe across concurrent @ref Run calls.
 */

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/common/network/common/options.hpp"
#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/common/network/detail/postprocess/boxes.hpp"
#include "autonomy/common/network/detail/postprocess/cls.hpp"
#include "autonomy/common/network/detail/postprocess/find.hpp"
#include "autonomy/common/network/detail/postprocess/map.hpp"
#include "autonomy/common/network/detail/postprocess/nms.hpp"
#include "autonomy/common/network/detail/postprocess/types.hpp"
#include "autonomy/common/network/pipeline/run.hpp"
#include "autonomy/common/network/preprocess.hpp"

#endif  // AUTONOMY_COMMON_NETWORK_NETWORK_HPP_
