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

#ifndef AUTONOMY_COMMON_NETWORK_COMMON_OPTIONS_HPP_
#define AUTONOMY_COMMON_NETWORK_COMMON_OPTIONS_HPP_

#include <optional>
#include <string>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file options.hpp
 * @brief Inference backend configuration (ONNX Runtime / TensorRT)
 *
 * Consumed by @ref Engine::CreateEngine and @ref Backend::LoadFromOptions.
 * `backend_id` selects which nested struct (`onnx` vs `tensorrt`) applies.
 */

/**
 * @brief ONNX Runtime session options
 *
 * Thread counts and graph optimization level of 0 / -1 mean ORT defaults.
 * @see https://onnxruntime.ai/docs/
 */
struct OnnxRuntimeOptions {
    int intra_op_num_threads = 0;  //!< Intra-op threads; 0 = ORT default
    int inter_op_num_threads = 0;  //!< Inter-op threads; 0 = ORT default
    int graph_optimization_level =
        -1;  //!< Graph opt level; -1 = default, 0–3 = ORT levels

    /**
     * @brief Execution provider: `""` or `"cpu"` (default), `"cuda"` for CUDA
     * EP
     *
     * Requires an ONNX Runtime build with the CUDA provider linked.
     */
    std::string execution_provider;

    int device_id = 0;  //!< GPU device when execution_provider is `"cuda"`

    /**
     * @brief Use ORT IoBinding with pre-allocated host output buffers (CPU EP)
     *
     * When true and every output has a fully static shape, outputs are written
     * directly into caller-owned @ref Tensor storage (no post-run memcpy).
     * Falls back to classic `Session::Run` when any output shape is dynamic.
     */
    bool use_io_binding = false;
};

/**
 * @brief TensorRT builder and runtime options
 *
 * Used when building an engine from ONNX or loading a serialized engine.
 */
struct TensorRtOptions {
    std::optional<int>
        device_id;  //!< CUDA device index; nullopt = default device
    int max_workspace_size_mb = 0;  //!< Builder workspace (MB); 0 = default
    std::string
        trt_cache_path;  //!< Directory for ONNX→engine cache (may be empty)
};

/**
 * @brief Unified load configuration for @ref Engine and @ref Backend
 *
 * @note `model_path` extension must match `backend_id`:
 *       - onnx: `.onnx`
 *       - tensorrt: `.onnx` (build at load) or `.engine` / `.plan` (serialized)
 */
struct InferenceOptions {
    std::string backend_id = "onnx";  //!< Factory id: `"onnx"` or `"tensorrt"`
    std::string model_path;           //!< Absolute or relative model path
    OnnxRuntimeOptions onnx;          //!< Used when backend_id is `"onnx"`
    TensorRtOptions tensorrt;         //!< Used when backend_id is `"tensorrt"`

    /**
     * @brief Overwrites fields from @p o (does not change backend_id unless set
     * in @p o)
     * @param o Source options
     */
    void Merge(const InferenceOptions& o);
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_COMMON_OPTIONS_HPP_
