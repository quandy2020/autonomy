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

#include "autonomy/common/network/rknn.hpp"

namespace autonomy {
namespace common {
namespace network {

bool RknnBackend::LoadFromFile(const std::string& model_path) {
    (void)model_path;
    SetLastError(
        "RknnBackend is a stub: link Rockchip rknn_api and load .rknn models to "
        "enable.");
    return false;
}

bool RknnBackend::LoadFromOptions(const InferenceOptions& opt) {
    return LoadFromFile(opt.model_path);
}

bool RknnBackend::IsLoaded() const {
    return false;
}

std::vector<ModelTensorInfo> RknnBackend::GetInputInfos() const {
    return {};
}

std::vector<ModelTensorInfo> RknnBackend::GetOutputInfos() const {
    return {};
}

bool RknnBackend::Run(
    const std::unordered_map<std::string, std::vector<float>>& inputs,
    std::unordered_map<std::string, std::vector<float>>* outputs) {
    (void)inputs;
    (void)outputs;
    SetLastError("RknnBackend::Run is not implemented.");
    return false;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
