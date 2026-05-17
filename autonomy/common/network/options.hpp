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

#ifndef AUTONOMY_COMMON_NETWORK_OPTIONS_HPP_
#define AUTONOMY_COMMON_NETWORK_OPTIONS_HPP_

#include <optional>
#include <string>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file options.hpp
 * @brief Inference backend configuration option structs
 */

/**
 * @brief ONNX Runtime session options used by the ONNX backend
 */
struct OnnxRuntimeOptions {
    int intra_op_num_threads = 0;  //!< @brief Intra-op threads; 0 = ORT default
    int inter_op_num_threads = 0;  //!< @brief Inter-op threads; 0 = ORT default
    int graph_optimization_level = -1;  //!< @brief ORT graph opt level; -1 = session default
};

/**
 * @brief TensorRT-specific options (used when `backend_id` is `"tensorrt"`).
 */
struct TensorRtOptions {
    std::optional<int> device_id;  //!< @brief CUDA device; unset = backend default
    int max_workspace_size_mb = 0;  //!< @brief Workspace size in MB; 0 = default
    std::string trt_cache_path;     //!< @brief Optional TensorRT engine cache directory
};

/**
 * @brief Rockchip RKNN-specific options (backend_id "rknn")
 */
struct RknnOptions {
    std::optional<int> device_id;  //!< @brief NPU device index
    std::optional<int> core_mask;  //!< @brief NPU core mask; unset = platform default
};

/**
 * @brief Unified configuration for @ref Engine and @ref Inference
 */
struct InferenceOptions {
    std::string backend_id = "onnx";  //!< @brief Factory key: "onnx", "tensorrt", "rknn", ...
    std::string model_path;           //!< @brief Path to model file on disk

    OnnxRuntimeOptions onnx;      //!< @brief Used when backend_id is onnx
    TensorRtOptions tensorrt;     //!< @brief Used when backend_id is tensorrt
    RknnOptions rknn;             //!< @brief Used when backend_id is rknn

    /**
     * @brief Overlay non-empty fields from @p o onto this object
     * @param o Source options (e.g. user overrides on top of defaults)
     */
    void Merge(const InferenceOptions& o);
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_OPTIONS_HPP_
