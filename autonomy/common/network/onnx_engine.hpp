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

#ifndef AUTONOMY_COMMON_NETWORK_ONNX_ENGINE_HPP_
#define AUTONOMY_COMMON_NETWORK_ONNX_ENGINE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * Tensor descriptor: shape (e.g. NCHW) and element type.
 * Used for querying model input/output metadata.
 */
struct TensorInfo {
    std::vector<int64_t> shape;
    enum class ElementType { kFloat32, kInt32, kInt64, kUint8, kFloat16 };
    ElementType type = ElementType::kFloat32;
};

/**
 * ONNX inference engine: load a model from file and run inference on CPU.
 *
 * Interface follows Google C++ style: snake_case for methods, clear ownership,
 * and bool return with optional error message for fallible operations.
 *
 * Example:
 *   OnnxEngine engine;
 *   if (!engine.LoadFromFile("model.onnx")) { ... }
 *   auto input = engine.GetInputNames();
 *   std::vector<float> data = ...;
 *   std::unordered_map<std::string, std::vector<float>> out;
 *   if (!engine.Infer({{input[0], data}}, &out)) { ... }
 */
class OnnxEngine
{
public:
    OnnxEngine() = default;
    ~OnnxEngine();

    // Non-copyable, movable (defined in .cpp so Impl stays incomplete in
    // headers).
    OnnxEngine(const OnnxEngine&) = delete;
    OnnxEngine& operator=(const OnnxEngine&) = delete;
    OnnxEngine(OnnxEngine&&) noexcept;
    OnnxEngine& operator=(OnnxEngine&&) noexcept;

    /**
     * Loads an ONNX model from a file path.
     * @param model_path Path to the .onnx file.
     * @return true on success; false on failure (e.g. file not found, invalid
     * ONNX).
     */
    bool LoadFromFile(const std::string& model_path);

    /**
     * Loads an ONNX model from in-memory bytes.
     * @param model_data Pointer to the first byte of the model.
     * @param size Size in bytes.
     * @return true on success; false on failure.
     */
    bool LoadFromMemory(const void* model_data, size_t size);

    /** Returns whether a model is currently loaded. */
    bool IsLoaded() const;

    /** Returns the number of model inputs. */
    size_t GetInputCount() const;

    /** Returns the number of model outputs. */
    size_t GetOutputCount() const;

    /**
     * Returns the ordered names of model inputs.
     * Empty if not loaded.
     */
    std::vector<std::string> GetInputNames() const;

    /**
     * Returns the ordered names of model outputs.
     * Empty if not loaded.
     */
    std::vector<std::string> GetOutputNames() const;

    /**
     * Returns metadata for the i-th input (0-based).
     * Returns nullopt if not loaded or index out of range.
     */
    std::optional<TensorInfo> GetInputInfo(size_t index) const;

    /**
     * Returns metadata for the i-th output (0-based).
     * Returns nullopt if not loaded or index out of range.
     */
    std::optional<TensorInfo> GetOutputInfo(size_t index) const;

    /**
     * Runs inference.
     * @param inputs Map from input name to contiguous float data (row-major).
     *               Sizes must match the model's expected input sizes.
     * @param outputs On success, filled with output name -> float data.
     * @return true on success; false on failure (e.g. wrong shape/count).
     */
    bool Infer(
        const std::unordered_map<std::string, std::vector<float>>& inputs,
        std::unordered_map<std::string, std::vector<float>>* outputs);

    /**
     * Runs inference using input/output name vectors and contiguous buffers.
     * Useful when names are known and order is fixed.
     * @param input_names Order of names matching input_data.
     * @param input_data One contiguous buffer per input (row-major float).
     * @param output_names Names of outputs to fetch.
     * @param outputs On success, same order as output_names, each a vector of
     * float.
     * @return true on success; false on failure.
     */
    bool Infer(const std::vector<std::string>& input_names,
               const std::vector<std::vector<float>>& input_data,
               const std::vector<std::string>& output_names,
               std::vector<std::vector<float>>* outputs);

    /** Last error message after a failed Load* or Run. */
    const std::string& GetLastError() const {
        return last_error_;
    }

private:
    struct Impl;
    struct ImplDeleter {
        void operator()(Impl* p) const;
    };
    std::unique_ptr<Impl, ImplDeleter> impl_;
    mutable std::string last_error_;
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_ONNX_ENGINE_HPP_
