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

#include "autonomy/common/network/backend/onnx/io.hpp"

#include <cstring>

namespace autonomy {
namespace common {
namespace network {
namespace onnx {
namespace detail {
namespace {

int64_t ShapeElementCount(const std::vector<int64_t>& shape) {
    int64_t count = 1;
    for (int64_t d : shape) {
        if (d <= 0) {
            return -1;
        }
        count *= d;
    }
    return count;
}

template <typename T>
Ort::Value MakeOrtTensor(Ort::MemoryInfo& memory_info, void* data, size_t count,
                         const std::vector<int64_t>& shape) {
    return Ort::Value::CreateTensor<T>(memory_info, reinterpret_cast<T*>(data), count,
                                       shape.data(), shape.size());
}

template <typename T>
bool CopyTypedOutput(const T* src, size_t count, ElementType element_type, Tensor* out,
                     std::string* error) {
    if (src == nullptr || out == nullptr) {
        if (error != nullptr) {
            *error = "CopyTypedOutput: null pointer.";
        }
        return false;
    }
    const size_t bytes = count * sizeof(T);
    std::vector<uint8_t> buffer(bytes);
    if (bytes > 0) {
        std::memcpy(buffer.data(), src, bytes);
    }
    *out = Tensor(element_type, std::move(buffer));
    return true;
}

}  // namespace

bool MapOrtElementType(ONNXTensorElementDataType ort_type, ElementType* out) {
    if (out == nullptr) {
        return false;
    }
    switch (ort_type) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            *out = ElementType::kFloat32;
            return true;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            *out = ElementType::kFloat16;
            return true;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
            *out = ElementType::kBfloat16;
            return true;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            *out = ElementType::kInt8;
            return true;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            *out = ElementType::kInt32;
            return true;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            *out = ElementType::kInt64;
            return true;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            *out = ElementType::kUint8;
            return true;
        default:
            return false;
    }
}

bool CreateOrtInputTensor(const ModelTensorInfo& meta, const Tensor& host,
                          Ort::MemoryInfo& memory_info, Ort::Value* out, std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "CreateOrtInputTensor: out is null.";
        }
        return false;
    }
    if (!IsRuntimeElementType(meta.element_type)) {
        if (error != nullptr) {
            *error = "Input \"" + meta.name + "\" has unsupported element type.";
        }
        return false;
    }
    if (host.element_type() != meta.element_type) {
        if (error != nullptr) {
            *error = "Input \"" + meta.name + "\" element type mismatch.";
        }
        return false;
    }

    std::vector<int64_t> shape;
    if (!ResolveShapeForElementCount(meta, host.element_count(), &shape, error)) {
        return false;
    }

    const size_t count = host.element_count();
    void* data = host.mutable_bytes();

    switch (meta.element_type) {
        case ElementType::kFloat32:
            *out = MakeOrtTensor<float>(memory_info, data, count, shape);
            return true;
        case ElementType::kFloat16:
        case ElementType::kBfloat16:
            *out = MakeOrtTensor<uint16_t>(memory_info, data, count, shape);
            return true;
        case ElementType::kInt8:
            *out = MakeOrtTensor<int8_t>(memory_info, data, count, shape);
            return true;
        case ElementType::kInt32:
            *out = MakeOrtTensor<int32_t>(memory_info, data, count, shape);
            return true;
        case ElementType::kInt64:
            *out = MakeOrtTensor<int64_t>(memory_info, data, count, shape);
            return true;
        case ElementType::kUint8:
            *out = MakeOrtTensor<uint8_t>(memory_info, data, count, shape);
            return true;
    }

    if (error != nullptr) {
        *error = "Input \"" + meta.name + "\": unsupported element type.";
    }
    return false;
}

bool CopyOrtOutputTensor(Ort::Value& ort_value, const ModelTensorInfo& meta, Tensor* out,
                         std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "CopyOrtOutputTensor: out is null.";
        }
        return false;
    }
    if (!ort_value.IsTensor()) {
        if (error != nullptr) {
            *error = "Output \"" + meta.name + "\" is not a tensor.";
        }
        return false;
    }

    Ort::TensorTypeAndShapeInfo info = ort_value.GetTensorTypeAndShapeInfo();
    ElementType element_type = meta.element_type;
    ElementType mapped = element_type;
    if (!MapOrtElementType(info.GetElementType(), &mapped)) {
        if (error != nullptr) {
            *error = "Output \"" + meta.name + "\" has unsupported ONNX element type " +
                     std::to_string(static_cast<int>(info.GetElementType())) + ".";
        }
        return false;
    }
    if (mapped != element_type) {
        element_type = mapped;
    }

    const std::vector<int64_t> shape = info.GetShape();
    const int64_t count = ShapeElementCount(shape);
    if (count < 0) {
        if (error != nullptr) {
            *error = "Output \"" + meta.name + "\" has dynamic/invalid shape.";
        }
        return false;
    }
    const size_t n = static_cast<size_t>(count);

    switch (element_type) {
        case ElementType::kFloat32:
            return CopyTypedOutput(ort_value.GetTensorData<float>(), n, element_type, out,
                                   error);
        case ElementType::kFloat16:
        case ElementType::kBfloat16:
            return CopyTypedOutput(ort_value.GetTensorData<uint16_t>(), n, element_type, out,
                                   error);
        case ElementType::kInt8:
            return CopyTypedOutput(ort_value.GetTensorData<int8_t>(), n, element_type, out,
                                   error);
        case ElementType::kInt32:
            return CopyTypedOutput(ort_value.GetTensorData<int32_t>(), n, element_type, out,
                                   error);
        case ElementType::kInt64:
            return CopyTypedOutput(ort_value.GetTensorData<int64_t>(), n, element_type, out,
                                   error);
        case ElementType::kUint8:
            return CopyTypedOutput(ort_value.GetTensorData<uint8_t>(), n, element_type, out,
                                   error);
        default:
            break;
    }

    if (error != nullptr) {
        *error = "Output \"" + meta.name + "\": unsupported element type.";
    }
    return false;
}

namespace {

bool AllocateStaticOutputTensor(const ModelTensorInfo& meta, Tensor* out, std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "AllocateStaticOutputTensor: out is null.";
        }
        return false;
    }
    if (!IsFullyStaticShape(meta.shape)) {
        if (error != nullptr) {
            *error = "Output \"" + meta.name + "\" does not have a fully static shape.";
        }
        return false;
    }
    const int64_t count = meta.shape.ProductPositiveDims();
    const size_t elem_bytes = ElementTypeByteSize(meta.element_type);
    if (count <= 0 || elem_bytes == 0) {
        if (error != nullptr) {
            *error = "Output \"" + meta.name + "\" has invalid static shape or element type.";
        }
        return false;
    }
    *out = Tensor(meta.element_type, std::vector<uint8_t>(static_cast<size_t>(count) * elem_bytes));
    return true;
}

bool RunClassicInference(Ort::Session& session,
                         const std::vector<ModelTensorInfo>& input_infos,
                         const std::vector<ModelTensorInfo>& output_infos,
                         const TensorMap& inputs, TensorMap* outputs, std::string* error) {
    std::vector<Ort::Value> input_tensors;
    std::vector<const char*> input_names;
    input_tensors.reserve(input_infos.size());
    input_names.reserve(input_infos.size());

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    for (const ModelTensorInfo& in : input_infos) {
        const auto it = inputs.find(in.name);
        if (it == inputs.end()) {
            if (error != nullptr) {
                *error = "Missing input tensor: " + in.name;
            }
            return false;
        }
        Ort::Value tensor;
        if (!CreateOrtInputTensor(in, it->second, mem_info, &tensor, error)) {
            return false;
        }
        input_names.push_back(in.name.c_str());
        input_tensors.push_back(std::move(tensor));
    }

    std::vector<const char*> output_names;
    output_names.reserve(output_infos.size());
    for (const ModelTensorInfo& o : output_infos) {
        output_names.push_back(o.name.c_str());
    }

    auto ort_outputs = session.Run(Ort::RunOptions{nullptr}, input_names.data(),
                                   input_tensors.data(), input_tensors.size(),
                                   output_names.data(), output_names.size());

    if (ort_outputs.size() != output_infos.size()) {
        if (error != nullptr) {
            *error = "Unexpected output count from ONNX Runtime.";
        }
        return false;
    }

    outputs->clear();
    for (size_t i = 0; i < ort_outputs.size(); ++i) {
        const ModelTensorInfo& meta = output_infos[i];
        Tensor host;
        if (!CopyOrtOutputTensor(ort_outputs[i], meta, &host, error)) {
            return false;
        }
        (*outputs)[meta.name] = std::move(host);
    }
    return true;
}

bool RunIoBindingInference(Ort::Session& session,
                           const std::vector<ModelTensorInfo>& input_infos,
                           const std::vector<ModelTensorInfo>& output_infos,
                           const TensorMap& inputs, TensorMap* outputs, std::string* error) {
    Ort::IoBinding binding(session);
    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    std::vector<Ort::Value> input_tensors;
    input_tensors.reserve(input_infos.size());
    for (const ModelTensorInfo& in : input_infos) {
        const auto it = inputs.find(in.name);
        if (it == inputs.end()) {
            if (error != nullptr) {
                *error = "Missing input tensor: " + in.name;
            }
            return false;
        }
        Ort::Value tensor;
        if (!CreateOrtInputTensor(in, it->second, mem_info, &tensor, error)) {
            return false;
        }
        binding.BindInput(in.name.c_str(), tensor);
        input_tensors.push_back(std::move(tensor));
    }

    std::vector<Tensor> output_hosts;
    std::vector<Ort::Value> output_tensors;
    output_hosts.reserve(output_infos.size());
    output_tensors.reserve(output_infos.size());

    for (const ModelTensorInfo& out : output_infos) {
        Tensor host;
        if (!AllocateStaticOutputTensor(out, &host, error)) {
            return false;
        }
        Ort::Value tensor;
        if (!CreateOrtInputTensor(out, host, mem_info, &tensor, error)) {
            return false;
        }
        binding.BindOutput(out.name.c_str(), tensor);
        output_hosts.push_back(std::move(host));
        output_tensors.push_back(std::move(tensor));
    }

    session.Run(Ort::RunOptions{nullptr}, binding);

    outputs->clear();
    for (size_t i = 0; i < output_infos.size(); ++i) {
        (*outputs)[output_infos[i].name] = std::move(output_hosts[i]);
    }
    return true;
}

}  // namespace

bool CanUseIoBinding(const std::vector<ModelTensorInfo>& output_infos) {
    if (output_infos.empty()) {
        return true;
    }
    for (const ModelTensorInfo& meta : output_infos) {
        if (!IsFullyStaticShape(meta.shape)) {
            return false;
        }
        if (meta.shape.ProductPositiveDims() <= 0) {
            return false;
        }
        if (!IsRuntimeElementType(meta.element_type)) {
            return false;
        }
    }
    return true;
}

bool RunInference(Ort::Session& session, const std::vector<ModelTensorInfo>& input_infos,
                  const std::vector<ModelTensorInfo>& output_infos, const TensorMap& inputs,
                  TensorMap* outputs, bool prefer_io_binding, std::string* error) {
    if (outputs == nullptr) {
        if (error != nullptr) {
            *error = "RunInference: outputs is null.";
        }
        return false;
    }
    if (prefer_io_binding && CanUseIoBinding(output_infos)) {
        return RunIoBindingInference(session, input_infos, output_infos, inputs, outputs, error);
    }
    return RunClassicInference(session, input_infos, output_infos, inputs, outputs, error);
}

}  // namespace detail
}  // namespace onnx
}  // namespace network
}  // namespace common
}  // namespace autonomy
