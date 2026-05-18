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

#include "autonomy/common/network/backend/onnx/onnx.hpp"

#include <onnxruntime_cxx_api.h>

#include <string>

#include "autonomy/common/network/backend/onnx/io.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {
namespace network {

using onnx::detail::MapOrtElementType;
using onnx::detail::RunInference;

namespace {

bool AppendOnnxExecutionProviders(Ort::SessionOptions& options,
                                  const OnnxRuntimeOptions& onnx_opts,
                                  std::string* error) {
    const std::string& ep = onnx_opts.execution_provider;
    if (ep.empty() || ep == "cpu") {
        return true;
    }
    if (ep == "cuda") {
        OrtCUDAProviderOptions cuda_options{};
        cuda_options.device_id = onnx_opts.device_id;
        try {
            options.AppendExecutionProvider_CUDA(cuda_options);
            return true;
        } catch (const Ort::Exception& e) {
            if (error != nullptr) {
                *error =
                    std::string("Failed to enable ONNX Runtime CUDA EP: ") +
                    e.what();
            }
            return false;
        }
    }
    if (error != nullptr) {
        *error = "Unknown onnx.execution_provider \"" + ep +
                 "\" (use \"cpu\" or \"cuda\").";
    }
    return false;
}

Ort::Env& OrtEnv() {
    // ERROR: suppress benign load-time warnings (e.g. constant_folding on Sqrt).
    static Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "autonomy_network");
    return env;
}

bool FillInfos(Ort::Session* session, bool is_input,
               Ort::AllocatorWithDefaultOptions& alloc,
               std::vector<ModelTensorInfo>* infos, std::string* error) {
    const size_t n =
        is_input ? session->GetInputCount() : session->GetOutputCount();
    infos->clear();
    infos->reserve(n);
    for (size_t i = 0; i < n; ++i) {
        Ort::AllocatedStringPtr name_ptr =
            is_input ? session->GetInputNameAllocated(i, alloc)
                     : session->GetOutputNameAllocated(i, alloc);
        Ort::TypeInfo type_info = is_input ? session->GetInputTypeInfo(i)
                                           : session->GetOutputTypeInfo(i);
        auto ts = type_info.GetTensorTypeAndShapeInfo();
        ElementType element_type = ElementType::kFloat32;
        if (!MapOrtElementType(ts.GetElementType(), &element_type)) {
            if (error != nullptr) {
                *error = std::string("Unsupported ONNX element type on ") +
                         (is_input ? "input" : "output") + " \"" +
                         name_ptr.get() + "\" (" +
                         std::to_string(static_cast<int>(ts.GetElementType())) +
                         ").";
            }
            return false;
        }
        std::vector<int64_t> shape = ts.GetShape();
        ModelTensorInfo mti;
        mti.name = std::string(name_ptr.get());
        mti.shape = TensorShape(std::move(shape));
        mti.element_type = element_type;
        infos->push_back(std::move(mti));
    }
    return true;
}

}  // namespace

struct OnnxBackend::Impl {
    Ort::SessionOptions session_options{};
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator{};
    std::vector<ModelTensorInfo> input_infos;
    std::vector<ModelTensorInfo> output_infos;
    bool use_io_binding = false;
};

OnnxBackend::OnnxBackend() : impl_(std::make_unique<Impl>()) {}

OnnxBackend::~OnnxBackend() = default;

OnnxBackend::OnnxBackend(OnnxBackend&&) noexcept = default;
OnnxBackend& OnnxBackend::operator=(OnnxBackend&&) noexcept = default;

bool OnnxBackend::LoadFromFile(const std::string& model_path) {
    InferenceOptions opt;
    opt.model_path = model_path;
    return LoadFromOptions(opt);
}

bool OnnxBackend::LoadFromOptions(const InferenceOptions& opt) {
    ClearLastError();
    if (opt.model_path.empty()) {
        SetLastError("InferenceOptions.model_path is empty.");
        return false;
    }

    try {
        impl_->session_options = Ort::SessionOptions{};
        impl_->session_options.SetSessionLogSeverityLevel(ORT_LOGGING_LEVEL_ERROR);
        if (opt.onnx.intra_op_num_threads > 0) {
            impl_->session_options.SetIntraOpNumThreads(
                opt.onnx.intra_op_num_threads);
        }
        if (opt.onnx.inter_op_num_threads > 0) {
            impl_->session_options.SetInterOpNumThreads(
                opt.onnx.inter_op_num_threads);
        }
        if (opt.onnx.graph_optimization_level >= 0) {
            impl_->session_options.SetGraphOptimizationLevel(
                static_cast<GraphOptimizationLevel>(
                    opt.onnx.graph_optimization_level));
        }
        std::string ep_err;
        if (!AppendOnnxExecutionProviders(impl_->session_options, opt.onnx,
                                          &ep_err)) {
            SetLastError(ep_err);
            return false;
        }
        impl_->use_io_binding = opt.onnx.use_io_binding;
        impl_->session = std::make_unique<Ort::Session>(
            OrtEnv(), opt.model_path.c_str(), impl_->session_options);
        std::string fill_err;
        if (!FillInfos(impl_->session.get(), true, impl_->allocator,
                       &impl_->input_infos, &fill_err) ||
            !FillInfos(impl_->session.get(), false, impl_->allocator,
                       &impl_->output_infos, &fill_err)) {
            impl_->session.reset();
            impl_->input_infos.clear();
            impl_->output_infos.clear();
            SetLastError(fill_err);
            return false;
        }
    } catch (const Ort::Exception& e) {
        impl_->session.reset();
        impl_->input_infos.clear();
        impl_->output_infos.clear();
        SetLastError(std::string("ONNX Runtime: ") + e.what());
        LOG(ERROR) << GetLastError();
        return false;
    }
    return true;
}

bool OnnxBackend::IsLoaded() const {
    return static_cast<bool>(impl_->session);
}

std::vector<ModelTensorInfo> OnnxBackend::GetInputInfos() const {
    return impl_->input_infos;
}

std::vector<ModelTensorInfo> OnnxBackend::GetOutputInfos() const {
    return impl_->output_infos;
}

bool OnnxBackend::Run(const TensorMap& inputs, TensorMap* outputs) {
    ClearLastError();
    if (outputs == nullptr) {
        SetLastError("outputs is null.");
        return false;
    }
    outputs->clear();

    if (!impl_->session) {
        SetLastError("No model loaded.");
        return false;
    }

    try {
        std::string run_err;
        if (!RunInference(*impl_->session, impl_->input_infos,
                          impl_->output_infos, inputs, outputs,
                          impl_->use_io_binding, &run_err)) {
            SetLastError(run_err);
            return false;
        }
    } catch (const Ort::Exception& e) {
        SetLastError(std::string("ONNX Runtime Run: ") + e.what());
        LOG(ERROR) << GetLastError();
        return false;
    }
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
