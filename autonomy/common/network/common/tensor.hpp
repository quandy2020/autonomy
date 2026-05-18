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

#ifndef AUTONOMY_COMMON_NETWORK_COMMON_TENSOR_HPP_
#define AUTONOMY_COMMON_NETWORK_COMMON_TENSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file tensor.hpp
 * @brief Runtime tensor buffers, shape metadata, and float map conversions
 *
 * Unified I/O model for the network module:
 *
 * - @ref ElementType — element type enum (aligned with ONNX / TensorRT
 * metadata)
 * - @ref Tensor — typed contiguous row-major byte buffer
 * - @ref TensorMap — name → tensor for @ref Engine::Run
 * - @ref ModelTensorInfo — model input/output descriptor (name, shape, type)
 *
 * Conventions:
 * - Storage is C-contiguous row-major; shape is described by @ref TensorShape
 * - Preprocess emits float32; backends pass FP16 / BF16 / INT8 unchanged when
 * matched
 * - For postprocess float views, prefer @ref Tensor::TryViewFloat32, else @ref
 * Tensor::ToFloat32
 */

/**
 * @brief Element types supported for model I/O and runtime buffers
 *
 * Aligned with ONNX Runtime `ONNXTensorElementDataType` and TensorRT bindings.
 * Not every value is pass-through capable; see @ref IsRuntimeElementType.
 */
enum class ElementType {
    kFloat32,   //!< 32-bit IEEE 754 single-precision float
    kFloat16,   //!< 16-bit half (stored as raw uint16 bytes)
    kBfloat16,  //!< 16-bit bfloat16 (stored as raw uint16 bytes)
    kInt8,      //!< 8-bit signed integer (common in quantized models)
    kInt32,     //!< 32-bit signed integer
    kInt64,     //!< 64-bit signed integer (e.g. shape / index tensors)
    kUint8,     //!< 8-bit unsigned integer
};

/**
 * @brief Byte size of one element; 0 if unknown
 * @param type Element type
 */
size_t ElementTypeByteSize(ElementType type);

/**
 * @brief Whether @ref Backend::Run can pass this type through unchanged
 *
 * @param type Type to check
 * @return true if current ONNX / TensorRT implementations support it
 */
bool IsRuntimeElementType(ElementType type);

/**
 * @brief Contiguous tensor buffer with an explicit element type
 *
 * Holds raw bytes in `std::vector<uint8_t>`; does not encode layout beyond
 * element type. Shape validation uses @ref ResolveShapeForElementCount and
 * backends at Run time.
 */
class Tensor
{
public:
    /** @brief Default-constructs an empty float32 tensor */
    Tensor() = default;

    /**
     * @brief Constructs from existing bytes
     * @param element_type Element type
     * @param bytes Raw bytes; length must be a multiple of the element size
     */
    Tensor(ElementType element_type, std::vector<uint8_t> bytes);

    /** @return Current element type */
    ElementType element_type() const {
        return element_type_;
    }

    /** @return Total byte count */
    size_t byte_size() const {
        return bytes_.size();
    }

    /**
     * @brief Element count (byte_size / element byte size)
     * @return See .cpp when size is not divisible
     */
    size_t element_count() const;

    /** @return Read-only bytes; may be nullptr for empty tensor */
    const uint8_t* bytes() const {
        return bytes_.data();
    }

    /** @return Writable bytes */
    uint8_t* mutable_bytes() {
        return bytes_.data();
    }

    /** @return Mutable underlying storage */
    std::vector<uint8_t>& mutable_storage() {
        return bytes_;
    }

    /** @return Const underlying storage */
    const std::vector<uint8_t>& storage() const {
        return bytes_;
    }

    /**
     * @brief Builds a float32 tensor from a float vector
     * @param data Elements in logical order
     */
    static Tensor FromFloat32(std::vector<float> data);

    /**
     * @brief Builds a tensor from raw bytes
     * @param element_type Element type
     * @param bytes Buffer; length must be a multiple of @ref
     * ElementTypeByteSize
     */
    static Tensor FromBytes(ElementType element_type,
                            std::vector<uint8_t> bytes);

    /**
     * @brief Reinterprets storage as a T array
     * @tparam T Must match @ref element_type() size
     * @return nullptr on type or alignment mismatch
     */
    template <typename T>
    const T* data_as() const;

    /**
     * @brief Non-owning float32 view
     *
     * @param[out] data Pointer to internal float data
     * @param[out] count Element count
     * @param[out] error Reason when not float32
     * @return true only for kFloat32 with valid storage
     */
    bool TryViewFloat32(const float** data, size_t* count,
                        std::string* error = nullptr) const;

    /**
     * @brief Converts to float32 vector (with conversion when needed)
     *
     * For kFloat32 tensors, prefer @ref TryViewFloat32 to avoid an extra copy.
     *
     * @param[out] error Conversion failure message
     * @return float32 elements; empty on failure with non-empty error
     */
    std::vector<float> ToFloat32(std::string* error = nullptr) const;

private:
    ElementType element_type_ = ElementType::kFloat32;
    std::vector<uint8_t> bytes_;
};

template <typename T>
const T* Tensor::data_as() const {
    const size_t es = ElementTypeByteSize(element_type_);
    if (es != sizeof(T) || bytes_.size() % sizeof(T) != 0) {
        return nullptr;
    }
    return reinterpret_cast<const T*>(bytes_.data());
}

/** @brief Name → tensor map for inference (matches ONNX input/output names) */
using TensorMap = std::unordered_map<std::string, Tensor>;

/** @brief float32-only map for legacy code and @ref Engine::Run overload */
using FloatTensorMap = std::unordered_map<std::string, std::vector<float>>;

/**
 * @brief Dimension vector wrapper (from ONNX Runtime / TensorRT metadata)
 *
 * Dynamic axes may appear as <= 0 placeholders; @ref ProductPositiveDims may be
 * 0 then.
 */
class TensorShape
{
public:
    TensorShape() = default;

    /** @brief Constructs from a dimension list */
    explicit TensorShape(std::vector<int64_t> dims) : dims_(std::move(dims)) {}

    /** @return Rank (number of dimensions) */
    size_t Rank() const {
        return dims_.size();
    }

    /** @return Const dimensions */
    const std::vector<int64_t>& Dims() const {
        return dims_;
    }

    /** @return Mutable dimensions (for resolved shapes) */
    std::vector<int64_t>& MutableDims() {
        return dims_;
    }

    /**
     * @brief Product of all strictly positive dimensions
     * @return Product; 0 if any dimension is <= 0
     */
    int64_t ProductPositiveDims() const;

private:
    std::vector<int64_t> dims_;
};

/**
 * @brief Metadata for one model input or output tensor
 *
 * Returned by @ref Engine::GetInputInfos / GetOutputInfos for preprocess and
 * postprocess.
 */
struct ModelTensorInfo {
    std::string name;   //!< ONNX/TRT tensor name
    TensorShape shape;  //!< Shape (may include dynamic dims)
    ElementType element_type =
        ElementType::kFloat32;  //!< Declared element type
};

/** @brief True when every dimension is a known positive integer */
bool IsFullyStaticShape(const TensorShape& shape);

/**
 * @brief Validates element count against static shape and fills resolved
 * dimensions
 *
 * When the model has dynamic axes but the buffer size is fixed, unknown dims
 * are inferred from @p element_count.
 *
 * @param meta Model tensor descriptor
 * @param element_count Caller-provided element count
 * @param[out] resolved_shape Resolved per-axis sizes
 * @param[out] error Mismatch message
 * @return true when validation passes
 */
bool ResolveShapeForElementCount(const ModelTensorInfo& meta,
                                 size_t element_count,
                                 std::vector<int64_t>* resolved_shape,
                                 std::string* error = nullptr);

/**
 * @brief Converts a float32 convenience map to @ref TensorMap (all kFloat32)
 *
 * @param inputs Input name → float vector
 * @param[out] out Typed map
 * @param[out] error Failure reason (e.g. null out)
 */
bool FromFloatTensorMap(const FloatTensorMap& inputs, TensorMap* out,
                        std::string* error = nullptr);

/**
 * @brief Converts @ref TensorMap to float32 convenience map (per-tensor
 * conversion)
 *
 * @param outputs Backend typed outputs
 * @param[out] out Float map
 * @param[out] error Message when an output cannot be converted
 */
bool ToFloatTensorMap(const TensorMap& outputs, FloatTensorMap* out,
                      std::string* error = nullptr);

/**
 * @brief Builds a @ref Tensor for model input from a float32 preprocess buffer
 *
 * Converts to the model's @p target_type (float32, float16, integer types).
 * Quantized INT8 models should use @ref Sample::named_tensors with calibrated
 * values.
 *
 * @param float_data Row-major preprocess output (typically float32 pipeline)
 * @param target_type Expected type from @ref ModelTensorInfo::element_type
 * @param[out] error Failure reason
 */
Tensor FromPreprocessFloat(std::vector<float> float_data,
                           ElementType target_type,
                           std::string* error = nullptr);

/**
 * @brief Verifies inputs match model names, element counts, and element types
 *
 * @param inputs Tensors to run
 * @param infos @ref Engine::GetInputInfos
 * @param[out] error First mismatch description
 */
bool ValidateInputsForModel(const TensorMap& inputs,
                            const std::vector<ModelTensorInfo>& infos,
                            std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_COMMON_TENSOR_HPP_
