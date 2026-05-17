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

#ifndef AUTONOMY_COMMON_NETWORK_TENSORRT_HPP_
#define AUTONOMY_COMMON_NETWORK_TENSORRT_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/backend.hpp"
#include "autonomy/common/network/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file tensorrt.hpp
 * @brief TensorRT backend stub (factory id "tensorrt")
 *
 * Methods fail until engine load/run is implemented in tensorrt.cpp.
 * Use @ref TensorRtOptions for device, workspace, and cache paths when enabled.
 */

/**
 * @brief @ref Backend slot for NVIDIA TensorRT inference
 */
class TensorRtBackend : public Backend {
public:
    /**
     * @brief Constructs an unloaded TensorRT backend stub.
     */
    TensorRtBackend() = default;

    /**
     * @brief Destroys the backend.
     */
    ~TensorRtBackend() override = default;

    /**
     * @brief Attempts to load a TensorRT engine from disk (not implemented).
     *
     * @param model_path Engine or plan path (format TBD when implemented).
     * @return Always false in the stub; sets @ref GetLastError.
     */
    bool LoadFromFile(const std::string& model_path) override;

    /**
     * @brief Loads using @ref InferenceOptions (forwards to @ref LoadFromFile).
     *
     * Reads `opt.tensorrt` when a real implementation is added.
     *
     * @param opt Configuration including `model_path` and TensorRT options.
     * @return Always false in the stub.
     */
    bool LoadFromOptions(const InferenceOptions& opt) override;

    /**
     * @brief Returns whether an engine is loaded.
     *
     * @return Always false in the stub.
     */
    bool IsLoaded() const override;

    /**
     * @brief Returns input tensor metadata.
     *
     * @return Empty vector in the stub.
     */
    std::vector<ModelTensorInfo> GetInputInfos() const override;

    /**
     * @brief Returns output tensor metadata.
     *
     * @return Empty vector in the stub.
     */
    std::vector<ModelTensorInfo> GetOutputInfos() const override;

    /**
     * @brief Executes inference (not implemented).
     *
     * @param inputs Unused in the stub.
     * @param outputs Unused in the stub.
     * @return Always false; sets @ref GetLastError.
     */
    bool Run(const std::unordered_map<std::string, std::vector<float>>& inputs,
             std::unordered_map<std::string, std::vector<float>>* outputs) override;
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_TENSORRT_HPP_
