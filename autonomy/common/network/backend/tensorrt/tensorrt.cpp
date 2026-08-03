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

#include "autonomy/common/network/backend/tensorrt/tensorrt.hpp"

#include "autonomy/common/network/common/tensor.hpp"

#include <fstream>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"

#ifdef AUTONOMY_HAS_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#if defined(AUTONOMY_HAS_TENSORRT_ONNX_PARSER)
#include <NvOnnxParser.h>
#endif
#endif

namespace autonomy {
namespace common {
namespace network {
namespace {

#ifdef AUTONOMY_HAS_TENSORRT

class TrtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            LOG(WARNING) << "TensorRT: " << msg;
        }
    }
};

TrtLogger& GetTrtLogger() {
    static TrtLogger logger;
    return logger;
}

bool EndsWith(const std::string& path, const std::string& suffix) {
    return path.size() >= suffix.size() &&
           path.compare(path.size() - suffix.size(), suffix.size(), suffix) ==
               0;
}

std::vector<int64_t> DimsToShape(const nvinfer1::Dims& dims) {
    std::vector<int64_t> shape(static_cast<size_t>(dims.nbDims));
    for (int i = 0; i < dims.nbDims; ++i) {
        shape[static_cast<size_t>(i)] = dims.d[i];
    }
    return shape;
}

int64_t Volume(const nvinfer1::Dims& dims) {
    int64_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        v *= dims.d[i];
    }
    return v;
}

bool ReadBinaryFile(const std::string& path, std::vector<char>* out) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        return false;
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        return false;
    }
    file.seekg(0, std::ios::beg);
    out->resize(static_cast<size_t>(size));
    return static_cast<bool>(file.read(out->data(), size));
}

#endif  // AUTONOMY_HAS_TENSORRT

}  // namespace

struct TensorRtBackend::Impl {
#ifdef AUTONOMY_HAS_TENSORRT
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t stream = nullptr;
    std::vector<ModelTensorInfo> input_infos;
    std::vector<ModelTensorInfo> output_infos;
    std::unordered_map<std::string, void*> device_buffers;
    std::unordered_map<std::string, size_t> device_capacity_bytes;
#endif
};

TensorRtBackend::TensorRtBackend() : impl_(std::make_unique<Impl>()) {}

TensorRtBackend::~TensorRtBackend() {
#ifdef AUTONOMY_HAS_TENSORRT
    if (impl_->stream != nullptr) {
        cudaStreamDestroy(impl_->stream);
        impl_->stream = nullptr;
    }
    for (auto& entry : impl_->device_buffers) {
        if (entry.second != nullptr) {
            cudaFree(entry.second);
        }
    }
#endif
}

TensorRtBackend::TensorRtBackend(TensorRtBackend&&) noexcept = default;
TensorRtBackend& TensorRtBackend::operator=(TensorRtBackend&&) noexcept =
    default;

bool TensorRtBackend::LoadFromFile(const std::string& model_path) {
    InferenceOptions opt;
    opt.model_path = model_path;
    return LoadFromOptions(opt);
}

bool TensorRtBackend::LoadFromOptions(const InferenceOptions& opt) {
    ClearLastError();
    if (opt.model_path.empty()) {
        SetLastError("InferenceOptions.model_path is empty.");
        return false;
    }

#ifndef AUTONOMY_HAS_TENSORRT
    (void)opt;
    SetLastError(
        "TensorRT is not available in this build (enable BUILD_TENSORRT and "
        "install TensorRT + CUDA).");
    return false;
#else
    const int device =
        opt.tensorrt.device_id.has_value() ? *opt.tensorrt.device_id : 0;
    if (cudaSetDevice(device) != cudaSuccess) {
        SetLastError("cudaSetDevice failed for TensorRT.");
        return false;
    }

    impl_->runtime.reset(nvinfer1::createInferRuntime(GetTrtLogger()));
    if (!impl_->runtime) {
        SetLastError("createInferRuntime failed.");
        return false;
    }

    std::vector<char> engine_blob;
    const std::string& path = opt.model_path;

    auto load_engine_blob = [&](const std::vector<char>& blob) -> bool {
        impl_->engine.reset(
            impl_->runtime->deserializeCudaEngine(blob.data(), blob.size()));
        if (!impl_->engine) {
            SetLastError("deserializeCudaEngine failed.");
            return false;
        }
        return true;
    };

    if (EndsWith(path, ".engine") || EndsWith(path, ".plan")) {
        if (!ReadBinaryFile(path, &engine_blob)) {
            SetLastError("Failed to read TensorRT engine file: " + path);
            return false;
        }
        if (!load_engine_blob(engine_blob)) {
            return false;
        }
    } else if (EndsWith(path, ".onnx")) {
#if defined(AUTONOMY_HAS_TENSORRT_ONNX_PARSER)
        std::string cache_path;
        if (!opt.tensorrt.trt_cache_path.empty()) {
            cache_path = opt.tensorrt.trt_cache_path;
            if (cache_path.back() != '/') {
                cache_path += '/';
            }
            const size_t slash = path.find_last_of("/\\");
            const std::string base =
                slash == std::string::npos ? path : path.substr(slash + 1);
            cache_path += base + ".engine";
            if (ReadBinaryFile(cache_path, &engine_blob) &&
                load_engine_blob(engine_blob)) {
                LOG(INFO) << "Loaded TensorRT engine from cache: "
                          << cache_path;
            } else {
                engine_blob.clear();
                impl_->engine.reset();
            }
        }

        if (!impl_->engine) {
            auto builder = std::unique_ptr<nvinfer1::IBuilder>(
                nvinfer1::createInferBuilder(GetTrtLogger()));
            if (!builder) {
                SetLastError("createInferBuilder failed.");
                return false;
            }
            const auto explicit_batch =
                1U << static_cast<uint32_t>(
                    nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
            auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
                builder->createNetworkV2(explicit_batch));
            auto parser = std::unique_ptr<nvonnxparser::IParser>(
                nvonnxparser::createParser(*network, GetTrtLogger()));
            if (!parser->parseFromFile(
                    path.c_str(),
                    static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
                SetLastError("nvonnxparser failed to parse: " + path);
                return false;
            }
            auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
                builder->createBuilderConfig());
            if (opt.tensorrt.max_workspace_size_mb > 0) {
                config->setMemoryPoolLimit(
                    nvinfer1::MemoryPoolType::kWORKSPACE,
                    static_cast<size_t>(opt.tensorrt.max_workspace_size_mb) *
                        1024ULL * 1024ULL);
            }
            auto plan = std::unique_ptr<nvinfer1::IHostMemory>(
                builder->buildSerializedNetwork(*network, *config));
            if (!plan) {
                SetLastError("buildSerializedNetwork failed.");
                return false;
            }
            engine_blob.assign(plan->data(), plan->data() + plan->size());
            if (!cache_path.empty()) {
                std::ofstream cache_file(cache_path, std::ios::binary);
                if (cache_file) {
                    cache_file.write(
                        engine_blob.data(),
                        static_cast<std::streamsize>(engine_blob.size()));
                    LOG(INFO) << "Wrote TensorRT engine cache: " << cache_path;
                }
            }
            if (!load_engine_blob(engine_blob)) {
                return false;
            }
        }
#else
        SetLastError(
            "TensorRT ONNX parser (nvonnxparser) is not linked. Provide a "
            "prebuilt .engine file or rebuild with nvonnxparser.");
        return false;
#endif
    } else {
        SetLastError(
            "TensorRT model_path must be .onnx, .engine, or .plan (got: " +
            path + ").");
        return false;
    }

    impl_->context.reset(impl_->engine->createExecutionContext());
    if (!impl_->context) {
        SetLastError("createExecutionContext failed.");
        return false;
    }
    if (cudaStreamCreate(&impl_->stream) != cudaSuccess) {
        SetLastError("cudaStreamCreate failed.");
        return false;
    }

    impl_->input_infos.clear();
    impl_->output_infos.clear();
    impl_->device_buffers.clear();
    impl_->device_capacity_bytes.clear();

    const int32_t nb_io = impl_->engine->getNbIOTensors();
    for (int32_t i = 0; i < nb_io; ++i) {
        const char* name = impl_->engine->getIOTensorName(i);
        const nvinfer1::TensorIOMode mode =
            impl_->engine->getTensorIOMode(name);
        nvinfer1::Dims dims = impl_->engine->getTensorShape(name);
        const nvinfer1::DataType data_type =
            impl_->engine->getTensorDataType(name);

        ModelTensorInfo info;
        info.name = name;
        info.shape = TensorShape(DimsToShape(dims));
        switch (data_type) {
            case nvinfer1::DataType::kFLOAT:
                info.element_type = ElementType::kFloat32;
                break;
            case nvinfer1::DataType::kHALF:
                info.element_type = ElementType::kFloat16;
                break;
            case nvinfer1::DataType::kBF16:
                info.element_type = ElementType::kBfloat16;
                break;
            case nvinfer1::DataType::kINT8:
                info.element_type = ElementType::kInt8;
                break;
            case nvinfer1::DataType::kINT32:
                info.element_type = ElementType::kInt32;
                break;
            case nvinfer1::DataType::kINT64:
                info.element_type = ElementType::kInt64;
                break;
            case nvinfer1::DataType::kUINT8:
                info.element_type = ElementType::kUint8;
                break;
            default:
                SetLastError(
                    std::string("Unsupported TensorRT data type for tensor ") +
                    name);
                return false;
        }

        const int64_t vol = Volume(dims);
        if (vol > 0) {
            void* device_ptr = nullptr;
            const size_t elem_bytes = ElementTypeByteSize(info.element_type);
            const size_t bytes = static_cast<size_t>(vol) * elem_bytes;
            if (cudaMalloc(&device_ptr, bytes) != cudaSuccess) {
                SetLastError(std::string("cudaMalloc failed for tensor ") +
                             name);
                return false;
            }
            impl_->device_buffers[name] = device_ptr;
            impl_->device_capacity_bytes[name] = bytes;
        }

        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            impl_->input_infos.push_back(std::move(info));
        } else {
            impl_->output_infos.push_back(std::move(info));
        }
    }
    return true;
#endif
}

bool TensorRtBackend::IsLoaded() const {
#ifdef AUTONOMY_HAS_TENSORRT
    return impl_->engine != nullptr && impl_->context != nullptr;
#else
    return false;
#endif
}

std::vector<ModelTensorInfo> TensorRtBackend::GetInputInfos() const {
#ifdef AUTONOMY_HAS_TENSORRT
    return impl_->input_infos;
#else
    return {};
#endif
}

std::vector<ModelTensorInfo> TensorRtBackend::GetOutputInfos() const {
#ifdef AUTONOMY_HAS_TENSORRT
    return impl_->output_infos;
#else
    return {};
#endif
}

bool TensorRtBackend::Run(const TensorMap& inputs, TensorMap* outputs) {
    ClearLastError();
    if (outputs == nullptr) {
        SetLastError("outputs is null.");
        return false;
    }
    outputs->clear();

#ifndef AUTONOMY_HAS_TENSORRT
    (void)inputs;
    SetLastError("TensorRT is not available in this build.");
    return false;
#else
    if (!impl_->context || !impl_->engine) {
        SetLastError("No TensorRT model loaded.");
        return false;
    }

    for (const ModelTensorInfo& in : impl_->input_infos) {
        const auto it = inputs.find(in.name());
        if (it == inputs.end()) {
            SetLastError("Missing input tensor: " + in.name());
            return false;
        }
        if (it->second.element_type() != in.element_type) {
            SetLastError("Input \"" + in.name() + "\" element type mismatch.");
            return false;
        }
        std::vector<int64_t> shape;
        std::string shape_err;
        if (!ResolveShapeForElementCount(in, it->second.element_count(), &shape,
                                         &shape_err)) {
            SetLastError(shape_err);
            return false;
        }
        nvinfer1::Dims dims{};
        dims.nbDims = static_cast<int>(shape.size());
        for (size_t i = 0; i < shape.size(); ++i) {
            dims.d[i] = static_cast<int>(shape[i]);
        }
        if (!impl_->context->setInputShape(in.name.c_str(), dims)) {
            SetLastError("setInputShape failed for " + in.name());
            return false;
        }

        const int64_t vol = std::accumulate(
            shape.begin(), shape.end(), int64_t{1}, std::multiplies<int64_t>());
        const size_t elem_bytes = ElementTypeByteSize(in.element_type);
        const size_t need_bytes = static_cast<size_t>(vol) * elem_bytes;
        auto buf_it = impl_->device_buffers.find(in.name());
        if (buf_it == impl_->device_buffers.end()) {
            SetLastError("No device buffer for input " + in.name());
            return false;
        }
        if (need_bytes > impl_->device_capacity_bytes[in.name()]) {
            SetLastError("Input size exceeds allocated device buffer for " +
                         in.name());
            return false;
        }
        if (it->second.byte_size() < need_bytes) {
            SetLastError("Input \"" + in.name() + "\" buffer too small.");
            return false;
        }
        if (cudaMemcpy(buf_it->second, it->second.bytes(), need_bytes,
                       cudaMemcpyHostToDevice) != cudaSuccess) {
            SetLastError("cudaMemcpy H2D failed for " + in.name());
            return false;
        }
        if (!impl_->context->setTensorAddress(in.name.c_str(),
                                              buf_it->second)) {
            SetLastError("setTensorAddress failed for " + in.name());
            return false;
        }
    }

    for (const ModelTensorInfo& out : impl_->output_infos) {
        nvinfer1::Dims dims = impl_->context->getTensorShape(out.name.c_str());
        const int64_t vol = Volume(dims);
        auto buf_it = impl_->device_buffers.find(out.name());
        if (buf_it == impl_->device_buffers.end()) {
            SetLastError("No device buffer for output " + out.name());
            return false;
        }
        const size_t elem_bytes = ElementTypeByteSize(out.element_type);
        const size_t need_bytes = static_cast<size_t>(vol) * elem_bytes;
        if (need_bytes > impl_->device_capacity_bytes[out.name()]) {
            SetLastError("Output size exceeds allocated device buffer for " +
                         out.name());
            return false;
        }
        if (!impl_->context->setTensorAddress(out.name.c_str(),
                                              buf_it->second)) {
            SetLastError("setTensorAddress failed for " + out.name());
            return false;
        }
    }

    if (!impl_->context->enqueueV3(impl_->stream)) {
        SetLastError("enqueueV3 failed.");
        return false;
    }
    if (cudaStreamSynchronize(impl_->stream) != cudaSuccess) {
        SetLastError("cudaStreamSynchronize failed.");
        return false;
    }

    for (const ModelTensorInfo& out : impl_->output_infos) {
        nvinfer1::Dims dims = impl_->context->getTensorShape(out.name.c_str());
        const int64_t vol = Volume(dims);
        if (vol <= 0) {
            SetLastError("Invalid output volume for " + out.name());
            return false;
        }
        const size_t elem_bytes = ElementTypeByteSize(out.element_type);
        const size_t need_bytes = static_cast<size_t>(vol) * elem_bytes;
        std::vector<uint8_t> host(need_bytes);
        void* device_ptr = impl_->device_buffers[out.name()];
        if (cudaMemcpy(host.data(), device_ptr, need_bytes,
                       cudaMemcpyDeviceToHost) != cudaSuccess) {
            SetLastError("cudaMemcpy D2H failed for " + out.name());
            return false;
        }
        (*outputs)[out.name()] = Tensor(out.element_type, std::move(host));
    }
    return true;
#endif
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
