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

#ifndef AUTONOMY_COMMON_NETWORK_RKNN_HPP_
#define AUTONOMY_COMMON_NETWORK_RKNN_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/backend.hpp"
#include "autonomy/common/network/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file rknn.hpp
 * @brief Rockchip RKNN backend stub (factory id "rknn")
 *
 * Methods fail until rknn_api integration in rknn.cpp. Intended for .rknn
 * models and @ref RknnOptions (device_id, core_mask).
 */

/**
 * @brief @ref Backend slot for Rockchip NPU inference via rknn_api
 */
class RknnBackend : public Backend {
public:
    /**
     * @brief Constructs an unloaded RKNN backend stub.
     */
    RknnBackend() = default;

    /**
     * @brief Destroys the backend.
     */
    ~RknnBackend() override = default;

    /**
     * @brief Attempts to load an RKNN model from disk (not implemented).
     *
     * @param model_path Path to a `.rknn` file when implemented.
     * @return Always false in the stub; sets @ref GetLastError.
     */
    bool LoadFromFile(const std::string& model_path) override;

    /**
     * @brief Loads using @ref InferenceOptions (forwards to @ref LoadFromFile).
     *
     * Will read `opt.rknn` when a real implementation is added.
     *
     * @param opt Configuration including `model_path` and RKNN options.
     * @return Always false in the stub.
     */
    bool LoadFromOptions(const InferenceOptions& opt) override;

    /**
     * @brief Returns whether an RKNN context is active.
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
     * @brief Executes inference on the NPU (not implemented).
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

#endif  // AUTONOMY_COMMON_NETWORK_RKNN_HPP_
