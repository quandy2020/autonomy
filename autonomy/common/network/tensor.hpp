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

#ifndef AUTONOMY_COMMON_NETWORK_TENSOR_HPP_
#define AUTONOMY_COMMON_NETWORK_TENSOR_HPP_

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
 * @brief Tensor shape metadata and I/O buffer types for network backends
 */

/** @brief Map from tensor name to contiguous row-major float32 data */
using TensorMap = std::unordered_map<std::string, std::vector<float>>;

/**
 * @brief Element types supported for model I/O metadata and validation
 */
enum class ElementType {
    kFloat32,  //!< @brief 32-bit IEEE float (primary runtime type)
    kFloat16,  //!< @brief 16-bit float
    kInt32,    //!< @brief 32-bit integer
    kInt64,    //!< @brief 64-bit integer
    kUint8,    //!< @brief 8-bit unsigned
};

/**
 * @brief Lightweight wrapper around a dimension vector from ONNX Runtime and similar
 *
 * Dimensions may be <= 0 when the runtime reports symbolic or unknown axes.
 */
class TensorShape {
public:
    /**
     * @brief Constructs an empty shape (rank zero).
     */
    TensorShape() = default;

    /**
     * @brief Constructs a shape with the given dimensions.
     *
     * @param dims Dimension list (e.g. NCHW for image models).
     */
    explicit TensorShape(std::vector<int64_t> dims) : dims_(std::move(dims)) {}

    /**
     * @brief Returns the number of dimensions.
     *
     * @return Rank of the tensor.
     */
    size_t Rank() const {
        return dims_.size();
    }

    /**
     * @brief Returns a read-only view of the dimension list.
     *
     * @return Const reference to stored dimensions (C++17).
     */
    const std::vector<int64_t>& Dims() const {
        return dims_;
    }

    /**
     * @brief Returns a mutable reference to the stored dimension vector.
     *
     * @return Reference to internal `std::vector<int64_t>`.
     */
    std::vector<int64_t>& MutableDims() {
        return dims_;
    }

    /**
     * @brief Alias for @ref Dims().
     */
    const std::vector<int64_t>& DimsVector() const {
        return dims_;
    }

    /**
     * @brief Multiplies all strictly positive dimensions.
     *
     * Unknown or dynamic entries (<= 0) are skipped. Useful as a partial
     * element-count hint when static dimensions are known.
     *
     * @return Product of positive dims, or 1 if none are positive.
     */
    int64_t ProductPositiveDims() const;

private:
    std::vector<int64_t> dims_;  //!< @brief Dimension list (NCHW, NHWC, etc.)
};

/**
 * @brief Describes one named model input or output tensor
 */
struct ModelTensorInfo {
    std::string name;  //!< @brief ONNX / runtime tensor name
    TensorShape shape; //!< @brief Static or partially dynamic shape
    ElementType element_type = ElementType::kFloat32;  //!< @brief Declared element type
};

/**
 * @brief Returns true if every dimension is strictly positive.
 *
 * @param shape Tensor shape to inspect.
 * @return True when rank > 0 and all dims > 0.
 */
bool IsFullyStaticShape(const TensorShape& shape);

/**
 * @brief Validates that `float_count` matches static positive dimensions.
 *
 * When the shape has at most one dynamic (<= 0) axis, that axis is inferred
 * from `float_count` and the product of known positive dimensions.
 *
 * @param meta Model tensor metadata (name used in error messages).
 * @param float_count Number of float elements supplied at runtime.
 * @param resolved_shape On success, concrete shape used for ORT tensors.
 * @param error Optional error message output.
 * @return True if the buffer size is consistent with the shape.
 */
bool ResolveShapeForFloatCount(const ModelTensorInfo& meta, size_t float_count,
                               std::vector<int64_t>* resolved_shape,
                               std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_TENSOR_HPP_
