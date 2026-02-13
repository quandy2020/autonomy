/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/common/network/onnx_engine.hpp"

#include <fstream>
#include <sstream>
#include <string>

#include <onnxruntime_cxx_api.h>
#include "autolink/common/log.hpp"

namespace autonomy {
namespace common {
namespace network {

namespace {

TensorInfo::ElementType FromOrtElementType(ONNXTensorElementDataType type) {
    switch (type) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return TensorInfo::ElementType::kFloat32;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            return TensorInfo::ElementType::kFloat16;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return TensorInfo::ElementType::kInt32;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            return TensorInfo::ElementType::kInt64;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return TensorInfo::ElementType::kUint8;
        default:
            return TensorInfo::ElementType::kFloat32;
    }
}

bool ReadFileToBuffer(const std::string& path, std::vector<char>* out) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        return false;
    }
    const auto size = static_cast<size_t>(f.tellg());
    f.seekg(0);
    out->resize(size);
    if (!f.read(out->data(), static_cast<std::streamsize>(size))) {
        return false;
    }
    return true;
}

}  // namespace

struct OnnxEngine::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "OnnxEngine"};
    Ort::Session session{nullptr};
    Ort::SessionOptions options;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<TensorInfo> input_infos;
    std::vector<TensorInfo> output_infos;

    Impl() {
        options.SetIntraOpNumThreads(1);
        options.SetLogSeverityLevel(3);
    }
};

OnnxEngine::~OnnxEngine() = default;

void OnnxEngine::ImplDeleter::operator()(Impl* p) const {
    delete p;
}

OnnxEngine::OnnxEngine(OnnxEngine&&) noexcept = default;

OnnxEngine& OnnxEngine::operator=(OnnxEngine&&) noexcept = default;

bool OnnxEngine::LoadFromFile(const std::string& model_path) {
    last_error_.clear();
    std::vector<char> buffer;
    if (!ReadFileToBuffer(model_path, &buffer)) {
        last_error_ = "Failed to read model file: " + model_path;
        AERROR << last_error_;
        return false;
    }
    return LoadFromMemory(buffer.data(), buffer.size());
}

bool OnnxEngine::LoadFromMemory(const void* model_data, size_t size) {
    last_error_.clear();
    impl_.reset(new Impl());
    try {
        impl_->session = Ort::Session(impl_->env, model_data, size, impl_->options);
    } catch (const Ort::Exception& e) {
        last_error_ = std::string("ONNX Runtime: ") + e.what();
        AERROR << last_error_;
        impl_.reset();
        return false;
    }

    Ort::AllocatorWithDefaultOptions allocator;
    const size_t num_inputs = impl_->session.GetInputCount();
    impl_->input_names.reserve(num_inputs);
    impl_->input_infos.reserve(num_inputs);

    for (size_t i = 0; i < num_inputs; ++i) {
#if ORT_API_VERSION >= 13
        auto name_ptr = impl_->session.GetInputNameAllocated(i, allocator);
        impl_->input_names.push_back(name_ptr.get());
#else
        const char* name = impl_->session.GetInputName(i, allocator);
        impl_->input_names.emplace_back(name);
        allocator.Free(const_cast<void*>(static_cast<const void*>(name)));
#endif
        auto type_info = impl_->session.GetInputTypeInfo(i);
        auto shape_info = type_info.GetTensorTypeAndShapeInfo();
        TensorInfo info;
        info.shape = shape_info.GetShape();
        info.type = FromOrtElementType(shape_info.GetElementType());
        impl_->input_infos.push_back(std::move(info));
    }

    const size_t num_outputs = impl_->session.GetOutputCount();
    impl_->output_names.reserve(num_outputs);
    impl_->output_infos.reserve(num_outputs);

    for (size_t i = 0; i < num_outputs; ++i) {
#if ORT_API_VERSION >= 13
        auto name_ptr = impl_->session.GetOutputNameAllocated(i, allocator);
        impl_->output_names.push_back(name_ptr.get());
#else
        const char* name = impl_->session.GetOutputName(i, allocator);
        impl_->output_names.emplace_back(name);
        allocator.Free(const_cast<void*>(static_cast<const void*>(name)));
#endif
        auto type_info = impl_->session.GetOutputTypeInfo(i);
        auto shape_info = type_info.GetTensorTypeAndShapeInfo();
        TensorInfo info;
        info.shape = shape_info.GetShape();
        info.type = FromOrtElementType(shape_info.GetElementType());
        impl_->output_infos.push_back(std::move(info));
    }

    return true;
}

bool OnnxEngine::IsLoaded() const {
    return impl_ != nullptr && impl_->session != nullptr;
}

size_t OnnxEngine::GetInputCount() const {
    return impl_ ? impl_->input_names.size() : 0;
}

size_t OnnxEngine::GetOutputCount() const {
    return impl_ ? impl_->output_names.size() : 0;
}

std::vector<std::string> OnnxEngine::GetInputNames() const {
    return impl_ ? impl_->input_names : std::vector<std::string>{};
}

std::vector<std::string> OnnxEngine::GetOutputNames() const {
    return impl_ ? impl_->output_names : std::vector<std::string>{};
}

std::optional<TensorInfo> OnnxEngine::GetInputInfo(size_t index) const {
    if (!impl_ || index >= impl_->input_infos.size()) {
        return std::nullopt;
    }
    return impl_->input_infos[index];
}

std::optional<TensorInfo> OnnxEngine::GetOutputInfo(size_t index) const {
    if (!impl_ || index >= impl_->output_infos.size()) {
        return std::nullopt;
    }
    return impl_->output_infos[index];
}

static size_t TensorSize(const std::vector<int64_t>& shape) {
    size_t n = 1;
    for (int64_t s : shape) {
        if (s <= 0) {
            return 0;
        }
        n *= static_cast<size_t>(s);
    }
    return n;
}

// Fills dynamic dimensions (-1) in shape so that product equals data_size.
// Returns false if impossible (e.g. multiple -1 with no way to infer).
static bool ConcretizeShape(std::vector<int64_t>* shape, size_t data_size) {
    int64_t product = 1;
    int dynamic_count = 0;
    for (int64_t s : *shape) {
        if (s == -1) {
            ++dynamic_count;
        } else if (s > 0) {
            product *= s;
        }
    }
    if (dynamic_count == 0) {
        return product == static_cast<int64_t>(data_size);
    }
    if (dynamic_count > 1) {
        return false;  // Cannot infer multiple dynamic dims.
    }
    if (product <= 0 || data_size % static_cast<size_t>(product) != 0) {
        return false;
    }
    int64_t inferred = static_cast<int64_t>(data_size / static_cast<size_t>(product));
    for (int64_t& s : *shape) {
        if (s == -1) {
            s = inferred;
            break;
        }
    }
    return true;
}

bool OnnxEngine::Infer(const std::unordered_map<std::string, std::vector<float>>& inputs,
                       std::unordered_map<std::string, std::vector<float>>* outputs) {
    if (!impl_ || !outputs) {
        if (!outputs) {
            last_error_ = "outputs is null";
        } else {
            last_error_ = "Model not loaded";
        }
        return false;
    }
    outputs->clear();

    std::vector<const char*> input_names_ptr;
    std::vector<std::vector<float>> input_storage;
    std::vector<std::vector<int64_t>> input_shapes;
    input_names_ptr.reserve(impl_->input_names.size());
    input_storage.reserve(impl_->input_names.size());
    input_shapes.reserve(impl_->input_names.size());

    for (size_t i = 0; i < impl_->input_names.size(); ++i) {
        const std::string& name = impl_->input_names[i];
        auto it = inputs.find(name);
        if (it == inputs.end()) {
            last_error_ = "Missing input: " + name;
            AERROR << last_error_;
            return false;
        }
        const std::vector<float>& data = it->second;
        const std::vector<int64_t>& shape = impl_->input_infos[i].shape;
        size_t expected = TensorSize(shape);
        if (expected != 0 && data.size() != expected) {
            std::ostringstream oss;
            oss << "Input \"" << name << "\" size mismatch: got " << data.size() << ", expected " << expected;
            last_error_ = oss.str();
            AERROR << last_error_;

            return false;
        }
        input_names_ptr.push_back(name.c_str());
        input_storage.push_back(data);
        std::vector<int64_t> shape_copy = shape;
        if (shape_copy.empty() && !data.empty()) {
            shape_copy.push_back(static_cast<int64_t>(data.size()));
        } else if (!shape_copy.empty()) {
            if (!ConcretizeShape(&shape_copy, data.size())) {
                last_error_ =
                    "Input \"" + name + "\": cannot infer dynamic shape from size " + std::to_string(data.size());
                return false;
            }
        }
        input_shapes.push_back(std::move(shape_copy));
    }

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    input_values.reserve(impl_->input_names.size());
    for (size_t i = 0; i < input_storage.size(); ++i) {
        Ort::Value value =
            Ort::Value::CreateTensor<float>(memory_info, input_storage[i].data(), input_storage[i].size(),
                                            input_shapes[i].data(), input_shapes[i].size());
        input_values.push_back(std::move(value));
    }

    std::vector<const char*> output_names_ptr;
    output_names_ptr.reserve(impl_->output_names.size());
    for (const std::string& n : impl_->output_names) {
        output_names_ptr.push_back(n.c_str());
    }

    try {
        Ort::RunOptions run_options;
        auto output_values =
            impl_->session.Run(run_options, input_names_ptr.data(), input_values.data(), input_names_ptr.size(),
                               output_names_ptr.data(), output_names_ptr.size());

        for (size_t i = 0; i < output_values.size(); ++i) {
            const Ort::Value& v = output_values[i];
            auto info = v.GetTensorTypeAndShapeInfo();
            ONNXTensorElementDataType elem_type = info.GetElementType();
            if (elem_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                last_error_ = "Unsupported output element type (only float32 supported for Infer)";
                return false;
            }
            size_t num_elements = static_cast<size_t>(info.GetElementCount());
            const float* data = v.GetTensorData<float>();
            std::string name = impl_->output_names[i];
            (*outputs)[name].assign(data, data + num_elements);
        }
    } catch (const Ort::Exception& e) {
        last_error_ = std::string("ONNX Runtime Infer: ") + e.what();
        AERROR << last_error_;
        return false;
    }
    return true;
}

bool OnnxEngine::Infer(const std::vector<std::string>& input_names, const std::vector<std::vector<float>>& input_data,
                       const std::vector<std::string>& output_names, std::vector<std::vector<float>>* outputs) {
    if (!impl_ || !outputs) {
        last_error_ = impl_ ? "outputs is null" : "Model not loaded";
        return false;
    }
    if (input_names.size() != input_data.size()) {
        last_error_ = "input_names and input_data size mismatch";
        return false;
    }
    std::unordered_map<std::string, std::vector<float>> input_map;
    for (size_t i = 0; i < input_names.size(); ++i) {
        input_map[input_names[i]] = input_data[i];
    }
    std::unordered_map<std::string, std::vector<float>> output_map;
    if (!Infer(input_map, &output_map)) {
        return false;
    }
    outputs->clear();
    outputs->reserve(output_names.size());
    for (const std::string& n : output_names) {
        auto it = output_map.find(n);
        if (it == output_map.end()) {
            last_error_ = "Output not found: " + n;
            return false;
        }
        outputs->push_back(it->second);
    }
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
