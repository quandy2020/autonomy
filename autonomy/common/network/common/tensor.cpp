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

#include "autonomy/common/network/common/tensor.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace autonomy {
namespace common {
namespace network {
namespace {

uint16_t FloatToHalf(float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    const uint32_t sign = (bits >> 16) & 0x8000u;
    uint32_t mantissa = bits & 0x007fffffu;
    int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xffu) - 127 + 15;

    if (exponent <= 0) {
        if (exponent < -10) {
            return static_cast<uint16_t>(sign);
        }
        mantissa |= 0x00800000u;
        const uint32_t shift = static_cast<uint32_t>(1 - exponent);
        mantissa >>= shift;
        return static_cast<uint16_t>(sign | (mantissa >> 13));
    }
    if (exponent >= 31) {
        return static_cast<uint16_t>(sign | 0x7c00u);
    }
    return static_cast<uint16_t>(sign |
                                 (static_cast<uint32_t>(exponent) << 10) |
                                 ((mantissa + 0x1000) >> 13));
}

uint16_t FloatToBfloat16(float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return static_cast<uint16_t>(bits >> 16);
}

float Bfloat16BitsToFloat(uint16_t bf) {
    const uint32_t bits = static_cast<uint32_t>(bf) << 16;
    float out = 0.f;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

float HalfBitsToFloat(uint16_t h) {
    const uint32_t sign = static_cast<uint32_t>((h >> 15) & 0x1u);
    uint32_t exp = static_cast<uint32_t>((h >> 10) & 0x1fu);
    uint32_t mant = static_cast<uint32_t>(h & 0x3ffu);
    uint32_t bits = 0;
    if (exp == 0) {
        if (mant == 0) {
            bits = sign << 31;
        } else {
            exp = 1;
            while ((mant & 0x400u) == 0) {
                mant <<= 1;
                --exp;
            }
            mant &= 0x3ffu;
            exp = 127 - 15 - exp;
            bits = (sign << 31) | (exp << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        bits = (sign << 31) | 0x7f800000u | (mant << 13);
    } else {
        exp = exp + 127 - 15;
        bits = (sign << 31) | (exp << 23) | (mant << 13);
    }
    float out = 0.f;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

template <typename T>
std::vector<float> CastToFloat32(const T* src, size_t count) {
    std::vector<float> out(count);
    for (size_t i = 0; i < count; ++i) {
        out[i] = static_cast<float>(src[i]);
    }
    return out;
}

}  // namespace

size_t ElementTypeByteSize(ElementType type) {
    switch (type) {
        case ElementType::kFloat32:
            return sizeof(float);
        case ElementType::kFloat16:
        case ElementType::kBfloat16:
            return sizeof(uint16_t);
        case ElementType::kInt8:
            return sizeof(int8_t);
        case ElementType::kInt32:
            return sizeof(int32_t);
        case ElementType::kInt64:
            return sizeof(int64_t);
        case ElementType::kUint8:
            return sizeof(uint8_t);
    }
    return 0;
}

bool IsRuntimeElementType(ElementType type) {
    return ElementTypeByteSize(type) > 0;
}

Tensor::Tensor(ElementType element_type, std::vector<uint8_t> bytes)
    : element_type_(element_type), bytes_(std::move(bytes)) {}

size_t Tensor::element_count() const {
    const size_t es = ElementTypeByteSize(element_type_);
    if (es == 0 || bytes_.size() % es != 0) {
        return 0;
    }
    return bytes_.size() / es;
}

Tensor Tensor::FromFloat32(std::vector<float> data) {
    std::vector<uint8_t> bytes(data.size() * sizeof(float));
    if (!data.empty()) {
        std::memcpy(bytes.data(), data.data(), bytes.size());
    }
    return Tensor(ElementType::kFloat32, std::move(bytes));
}

Tensor Tensor::FromBytes(ElementType element_type, std::vector<uint8_t> bytes) {
    return Tensor(element_type, std::move(bytes));
}

bool Tensor::TryViewFloat32(const float** data, size_t* count,
                            std::string* error) const {
    if (data == nullptr || count == nullptr) {
        if (error != nullptr) {
            *error = "TryViewFloat32: output pointers are null.";
        }
        return false;
    }
    if (element_type_ != ElementType::kFloat32) {
        if (error != nullptr) {
            *error = "tensor is not float32; use ToFloat32().";
        }
        return false;
    }
    const size_t n = element_count();
    *data = reinterpret_cast<const float*>(bytes_.data());
    *count = n;
    return true;
}

std::vector<float> Tensor::ToFloat32(std::string* error) const {
    const size_t n = element_count();
    if (n == 0 && !bytes_.empty()) {
        if (error != nullptr) {
            *error = "invalid tensor byte size for element type.";
        }
        return {};
    }
    switch (element_type_) {
        case ElementType::kFloat32:
            return std::vector<float>(data_as<float>(), data_as<float>() + n);
        case ElementType::kFloat16: {
            const uint16_t* src = data_as<uint16_t>();
            if (src == nullptr) {
                if (error != nullptr) {
                    *error = "invalid float16 tensor buffer.";
                }
                return {};
            }
            std::vector<float> out(n);
            for (size_t i = 0; i < n; ++i) {
                out[i] = HalfBitsToFloat(src[i]);
            }
            return out;
        }
        case ElementType::kBfloat16: {
            const uint16_t* src = data_as<uint16_t>();
            if (src == nullptr) {
                if (error != nullptr) {
                    *error = "invalid bfloat16 tensor buffer.";
                }
                return {};
            }
            std::vector<float> out(n);
            for (size_t i = 0; i < n; ++i) {
                out[i] = Bfloat16BitsToFloat(src[i]);
            }
            return out;
        }
        case ElementType::kInt8:
            return CastToFloat32(data_as<int8_t>(), n);
        case ElementType::kUint8:
            return CastToFloat32(data_as<uint8_t>(), n);
        case ElementType::kInt32:
            return CastToFloat32(data_as<int32_t>(), n);
        case ElementType::kInt64:
            return CastToFloat32(data_as<int64_t>(), n);
    }
    if (error != nullptr) {
        *error = "unsupported element type for ToFloat32().";
    }
    return {};
}

int64_t TensorShape::ProductPositiveDims() const {
    int64_t p = 1;
    for (int64_t d : dims_) {
        if (d <= 0) {
            return 0;
        }
        p *= d;
    }
    return p;
}

bool IsFullyStaticShape(const TensorShape& shape) {
    if (shape.Rank() == 0) {
        return false;
    }
    for (int64_t d : shape.Dims()) {
        if (d <= 0) {
            return false;
        }
    }
    return true;
}

bool ResolveShapeForElementCount(const ModelTensorInfo& meta,
                                 size_t element_count,
                                 std::vector<int64_t>* resolved_shape,
                                 std::string* error) {
    if (resolved_shape == nullptr) {
        if (error != nullptr) {
            *error = "ResolveShapeForElementCount: resolved_shape is null.";
        }
        return false;
    }
    const std::vector<int64_t>& dims = meta.shape.Dims();
    std::vector<int64_t> out(dims.begin(), dims.end());
    int64_t known = 1;
    int dyn_index = -1;
    for (size_t i = 0; i < out.size(); ++i) {
        if (out[i] <= 0) {
            if (dyn_index >= 0) {
                if (error != nullptr) {
                    *error = "Model tensor \"" + meta.name +
                             "\" has multiple dynamic dimensions.";
                }
                return false;
            }
            dyn_index = static_cast<int>(i);
        } else {
            known *= out[i];
        }
    }
    if (dyn_index >= 0) {
        if (known <= 0 || element_count % static_cast<size_t>(known) != 0) {
            if (error != nullptr) {
                *error = "Input \"" + meta.name +
                         "\": element count does not match model shape.";
            }
            return false;
        }
        out[static_cast<size_t>(dyn_index)] =
            static_cast<int64_t>(element_count / static_cast<size_t>(known));
    } else {
        if (known <= 0 || static_cast<size_t>(known) != element_count) {
            if (error != nullptr) {
                *error = "Input \"" + meta.name + "\": expected " +
                         std::to_string(known) + " elements, got " +
                         std::to_string(element_count);
            }
            return false;
        }
    }
    *resolved_shape = std::move(out);
    return true;
}

Tensor FromPreprocessFloat(std::vector<float> float_data,
                           ElementType target_type, std::string* error) {
    if (float_data.empty()) {
        if (error != nullptr) {
            *error = "FromPreprocessFloat: empty buffer.";
        }
        return {};
    }
    switch (target_type) {
        case ElementType::kFloat32:
            return Tensor::FromFloat32(std::move(float_data));
        case ElementType::kFloat16: {
            std::vector<uint8_t> bytes(float_data.size() * sizeof(uint16_t));
            uint16_t* dst = reinterpret_cast<uint16_t*>(bytes.data());
            for (size_t i = 0; i < float_data.size(); ++i) {
                dst[i] = FloatToHalf(float_data[i]);
            }
            return Tensor(ElementType::kFloat16, std::move(bytes));
        }
        case ElementType::kBfloat16: {
            std::vector<uint8_t> bytes(float_data.size() * sizeof(uint16_t));
            uint16_t* dst = reinterpret_cast<uint16_t*>(bytes.data());
            for (size_t i = 0; i < float_data.size(); ++i) {
                dst[i] = FloatToBfloat16(float_data[i]);
            }
            return Tensor(ElementType::kBfloat16, std::move(bytes));
        }
        case ElementType::kInt8: {
            std::vector<uint8_t> bytes(float_data.size());
            int8_t* dst = reinterpret_cast<int8_t*>(bytes.data());
            for (size_t i = 0; i < float_data.size(); ++i) {
                const float v = float_data[i];
                const float clamped = std::max(-128.f, std::min(127.f, v));
                dst[i] = static_cast<int8_t>(std::lround(clamped));
            }
            return Tensor(ElementType::kInt8, std::move(bytes));
        }
        case ElementType::kUint8: {
            std::vector<uint8_t> bytes(float_data.size());
            for (size_t i = 0; i < float_data.size(); ++i) {
                const float v = float_data[i];
                const float clamped = std::max(0.f, std::min(255.f, v));
                bytes[i] = static_cast<uint8_t>(std::lround(clamped));
            }
            return Tensor(ElementType::kUint8, std::move(bytes));
        }
        case ElementType::kInt32: {
            std::vector<uint8_t> bytes(float_data.size() * sizeof(int32_t));
            int32_t* dst = reinterpret_cast<int32_t*>(bytes.data());
            for (size_t i = 0; i < float_data.size(); ++i) {
                dst[i] = static_cast<int32_t>(std::lround(float_data[i]));
            }
            return Tensor(ElementType::kInt32, std::move(bytes));
        }
        case ElementType::kInt64: {
            std::vector<uint8_t> bytes(float_data.size() * sizeof(int64_t));
            int64_t* dst = reinterpret_cast<int64_t*>(bytes.data());
            for (size_t i = 0; i < float_data.size(); ++i) {
                dst[i] = static_cast<int64_t>(std::llround(float_data[i]));
            }
            return Tensor(ElementType::kInt64, std::move(bytes));
        }
    }
    if (error != nullptr) {
        *error = "FromPreprocessFloat: unsupported target element type.";
    }
    return {};
}

bool ValidateInputsForModel(const TensorMap& inputs,
                            const std::vector<ModelTensorInfo>& infos,
                            std::string* error) {
    for (const ModelTensorInfo& info : infos) {
        const auto it = inputs.find(info.name);
        if (it == inputs.end()) {
            if (error != nullptr) {
                *error = "Missing input tensor: " + info.name;
            }
            return false;
        }
        if (it->second.element_type() != info.element_type) {
            if (error != nullptr) {
                *error = "Input \"" + info.name + "\" element type mismatch.";
            }
            return false;
        }
        std::vector<int64_t> resolved;
        if (!ResolveShapeForElementCount(info, it->second.element_count(),
                                         &resolved, error)) {
            return false;
        }
    }
    return true;
}

bool FromFloatTensorMap(const FloatTensorMap& inputs, TensorMap* out,
                        std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "FromFloatTensorMap: out is null.";
        }
        return false;
    }
    out->clear();
    for (const auto& [name, vec] : inputs) {
        (void)name;
        (*out)[name] = Tensor::FromFloat32(vec);
    }
    return true;
}

bool ToFloatTensorMap(const TensorMap& outputs, FloatTensorMap* out,
                      std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "ToFloatTensorMap: out is null.";
        }
        return false;
    }
    out->clear();
    for (const auto& [name, tensor] : outputs) {
        std::string conv_err;
        std::vector<float> floats = tensor.ToFloat32(&conv_err);
        if (!conv_err.empty()) {
            if (error != nullptr) {
                *error = "Output \"" + name + "\": " + conv_err;
            }
            return false;
        }
        (*out)[name] = std::move(floats);
    }
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
