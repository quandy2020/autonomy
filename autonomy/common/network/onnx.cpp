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

#include "autonomy/common/network/onnx.hpp"

#include <string>

#include "glog/logging.h"

#ifdef AUTONOMY_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace autonomy {
namespace common {
namespace network {
namespace {

#ifdef AUTONOMY_HAS_ONNXRUNTIME

Ort::Env& OrtEnv() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "autonomy_network");
    return env;
}

ElementType MapElementType(ONNXTensorElementDataType t) {
    switch (t) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return ElementType::kFloat32;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            return ElementType::kFloat16;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return ElementType::kInt32;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            return ElementType::kInt64;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return ElementType::kUint8;
        default:
            return ElementType::kFloat32;
    }
}

void FillInfos(Ort::Session* session, bool is_input,
               Ort::AllocatorWithDefaultOptions& alloc,
               std::vector<ModelTensorInfo>* infos) {
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
        Ort::TensorTypeAndShapeInfo ts = type_info.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> shape = ts.GetShape();
        ModelTensorInfo mti;
        mti.name = std::string(name_ptr.get());
        mti.shape = TensorShape(std::move(shape));
        mti.element_type = MapElementType(ts.GetElementType());
        infos->push_back(std::move(mti));
    }
}

#endif  // AUTONOMY_HAS_ONNXRUNTIME

}  // namespace

struct OnnxBackend::Impl {
#ifdef AUTONOMY_HAS_ONNXRUNTIME
    Ort::SessionOptions session_options{};
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator{};
    std::vector<ModelTensorInfo> input_infos;
    std::vector<ModelTensorInfo> output_infos;
#endif
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

#ifndef AUTONOMY_HAS_ONNXRUNTIME
    (void)opt;
    SetLastError(
        "ONNX Runtime is not available in this build (define "
        "AUTONOMY_HAS_ONNXRUNTIME / enable BUILD_ONNXRUNTIME).");
    return false;
#else
    try {
        impl_->session_options = Ort::SessionOptions{};
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
        impl_->session = std::make_unique<Ort::Session>(
            OrtEnv(), opt.model_path.c_str(), impl_->session_options);
        FillInfos(impl_->session.get(), true, impl_->allocator,
                  &impl_->input_infos);
        FillInfos(impl_->session.get(), false, impl_->allocator,
                  &impl_->output_infos);
    } catch (const Ort::Exception& e) {
        impl_->session.reset();
        impl_->input_infos.clear();
        impl_->output_infos.clear();
        SetLastError(std::string("ONNX Runtime: ") + e.what());
        LOG(ERROR) << GetLastError();
        return false;
    }
    return true;
#endif
}

bool OnnxBackend::IsLoaded() const {
#ifdef AUTONOMY_HAS_ONNXRUNTIME
    return static_cast<bool>(impl_->session);
#else
    return false;
#endif
}

std::vector<ModelTensorInfo> OnnxBackend::GetInputInfos() const {
#ifdef AUTONOMY_HAS_ONNXRUNTIME
    return impl_->input_infos;
#else
    return {};
#endif
}

std::vector<ModelTensorInfo> OnnxBackend::GetOutputInfos() const {
#ifdef AUTONOMY_HAS_ONNXRUNTIME
    return impl_->output_infos;
#else
    return {};
#endif
}

bool OnnxBackend::Run(
    const std::unordered_map<std::string, std::vector<float>>& inputs,
    std::unordered_map<std::string, std::vector<float>>* outputs) {
    ClearLastError();
    if (!outputs) {
        SetLastError("outputs is null.");
        return false;
    }
    outputs->clear();

#ifndef AUTONOMY_HAS_ONNXRUNTIME
    (void)inputs;
    SetLastError("ONNX Runtime is not available in this build.");
    return false;
#else
    if (!impl_->session) {
        SetLastError("No model loaded.");
        return false;
    }

    try {
        std::vector<Ort::Value> input_tensors;
        std::vector<const char*> input_names;
        input_tensors.reserve(impl_->input_infos.size());
        input_names.reserve(impl_->input_infos.size());

        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator, OrtMemTypeDefault);

        for (const ModelTensorInfo& in : impl_->input_infos) {
            if (in.element_type != ElementType::kFloat32) {
                SetLastError("Input \"" + in.name +
                             "\" is not float32; use a typed API extension.");
                return false;
            }
            auto it = inputs.find(in.name);
            if (it == inputs.end()) {
                SetLastError("Missing input tensor: " + in.name);
                return false;
            }
            std::vector<int64_t> shape;
            std::string shape_err;
            if (!ResolveShapeForFloatCount(in, it->second.size(), &shape,
                                           &shape_err)) {
                SetLastError(std::move(shape_err));
                return false;
            }
            input_names.push_back(in.name.c_str());
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                mem_info, const_cast<float*>(it->second.data()),
                it->second.size(), shape.data(), shape.size()));
        }

        std::vector<const char*> output_names;
        output_names.reserve(impl_->output_infos.size());
        for (const ModelTensorInfo& o : impl_->output_infos) {
            output_names.push_back(o.name.c_str());
        }

        auto ort_outputs = impl_->session->Run(
            Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(),
            input_tensors.size(), output_names.data(), output_names.size());

        if (ort_outputs.size() != impl_->output_infos.size()) {
            SetLastError("Unexpected output count from ONNX Runtime.");
            return false;
        }

        for (size_t i = 0; i < ort_outputs.size(); ++i) {
            const std::string& out_name = impl_->output_infos[i].name;
            Ort::Value& val = ort_outputs[i];
            if (val.IsTensor() == 0) {
                SetLastError("Output \"" + out_name + "\" is not a tensor.");
                return false;
            }
            auto info = val.GetTensorTypeAndShapeInfo();
            ONNXTensorElementDataType ot = info.GetElementType();
            if (ot != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                SetLastError("Output \"" + out_name +
                             "\" is not float32; extend Run() for this type.");
                return false;
            }
            std::vector<int64_t> oshape = info.GetShape();
            int64_t count = 1;
            for (int64_t d : oshape) {
                count *= d;
            }
            const float* data = val.GetTensorData<float>();
            (*outputs)[out_name] =
                std::vector<float>(data, data + static_cast<size_t>(count));
        }
    } catch (const Ort::Exception& e) {
        SetLastError(std::string("ONNX Runtime Run: ") + e.what());
        LOG(ERROR) << GetLastError();
        return false;
    }
    return true;
#endif
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
