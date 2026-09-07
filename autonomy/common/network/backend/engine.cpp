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

#include "autonomy/common/network/backend/engine.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"

#include <utility>

#include "autonomy/common/network/backend/backend.hpp"
#include "autonomy/common/network/backend/factory.hpp"

namespace autonomy {
namespace common {
namespace network {

namespace {
using internal::SetErrorMessage;
}  // namespace

Engine::Engine() = default;

Engine::~Engine() = default;

Engine::Engine(Engine&&) noexcept = default;
Engine& Engine::operator=(Engine&&) noexcept = default;

Engine::Engine(std::unique_ptr<Backend> backend, std::string backend_id)
    : backend_(std::move(backend)), backend_id_(std::move(backend_id)) {}

std::unique_ptr<Engine> Engine::CreateEngine(const InferenceOptions& opt,
                                             std::string* error_message) {
    std::string err;
    std::unique_ptr<Backend> backend = BackendFactory::Create(
        opt, error_message != nullptr ? error_message : &err);
    if (!backend) {
        return nullptr;
    }
    const std::string id = opt.backend_id.empty() ? "onnx" : opt.backend_id;
    return std::unique_ptr<Engine>(new Engine(std::move(backend), id));
}

std::unique_ptr<Engine> Engine::CreateEngine(const std::string& model_path,
                                             const std::string& backend_id,
                                             std::string* error_message) {
    InferenceOptions opt;
    opt.model_path = model_path;
    opt.backend_id = backend_id;
    return CreateEngine(opt, error_message);
}

bool Engine::HasBackend(const std::string& backend_id) {
    return BackendFactory::HasBackend(backend_id);
}

bool Engine::LoadFromFile(const std::string& model_path) {
    last_error_.clear();
    if (!backend_) {
        last_error_ = "Engine has no backend; call CreateEngine first.";
        return false;
    }
    if (!backend_->LoadFromFile(model_path)) {
        last_error_ = backend_->GetLastError();
        return false;
    }
    return true;
}

bool Engine::LoadFromOptions(const InferenceOptions& opt) {
    last_error_.clear();
    if (!backend_) {
        last_error_ = "Engine has no backend; call CreateEngine first.";
        return false;
    }
    if (!opt.backend_id.empty()) {
        backend_id_ = opt.backend_id;
    }
    if (!backend_->LoadFromOptions(opt)) {
        last_error_ = backend_->GetLastError();
        return false;
    }
    return true;
}

bool Engine::IsLoaded() const {
    return backend_ && backend_->IsLoaded();
}

std::vector<ModelTensorInfo> Engine::GetInputInfos() const {
    if (!backend_) {
        return {};
    }
    return backend_->GetInputInfos();
}

std::vector<ModelTensorInfo> Engine::GetOutputInfos() const {
    if (!backend_) {
        return {};
    }
    return backend_->GetOutputInfos();
}

bool Engine::Run(const TensorMap& inputs, TensorMap* outputs) {
    last_error_.clear();
    if (!backend_) {
        last_error_ = "Engine has no backend.";
        return false;
    }
    if (outputs == nullptr) {
        last_error_ = "outputs is null.";
        return false;
    }
    std::string validate_err;
    if (!ValidateInputsForModel(inputs, GetInputInfos(), &validate_err)) {
        last_error_ = validate_err;
        return false;
    }
    if (!backend_->Run(inputs, outputs)) {
        last_error_ = backend_->GetLastError();
        return false;
    }
    return true;
}

bool Engine::Run(const FloatTensorMap& inputs, FloatTensorMap* outputs) {
    last_error_.clear();
    if (!backend_) {
        last_error_ = "Engine has no backend.";
        return false;
    }
    if (outputs == nullptr) {
        last_error_ = "outputs is null.";
        return false;
    }
    if (!backend_->Run(inputs, outputs)) {
        last_error_ = backend_->GetLastError();
        return false;
    }
    return true;
}

bool Engine::Warmup(std::string* error) {
    last_error_.clear();
    if (!backend_) {
        SetErrorMessage(error, "Engine has no backend.");
        last_error_ = error != nullptr ? *error : last_error_;
        return false;
    }
    if (!IsLoaded()) {
        SetErrorMessage(error, "Engine has no loaded model.");
        last_error_ = error != nullptr ? *error : last_error_;
        return false;
    }

    const std::vector<ModelTensorInfo> input_infos = GetInputInfos();
    for (const ModelTensorInfo& info : input_infos) {
        if (!IsFullyStaticShape(info.shape)) {
            return true;
        }
    }

    TensorMap inputs;
    for (const ModelTensorInfo& info : input_infos) {
        const int64_t count = info.shape.ProductPositiveDims();
        if (count <= 0) {
            continue;
        }
        const size_t byte_size =
            static_cast<size_t>(count) * ElementTypeByteSize(info.element_type);
        inputs.emplace(info.name, Tensor(info.element_type,
                                         std::vector<uint8_t>(byte_size)));
    }
    if (inputs.empty()) {
        return true;
    }

    TensorMap outputs;
    if (!Run(inputs, &outputs)) {
        if (error != nullptr) {
            *error = last_error_;
        }
        return false;
    }
    return true;
}

const std::string& Engine::GetLastError() const {
    if (!last_error_.empty()) {
        return last_error_;
    }
    if (backend_) {
        return backend_->GetLastError();
    }
    static const std::string kEmpty;
    return kEmpty;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
