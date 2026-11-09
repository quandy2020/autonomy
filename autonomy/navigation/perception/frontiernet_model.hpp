/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file frontiernet_model.hpp
 * @brief FrontierNet (DeFM encoder) inference on autonomy::common::network::Engine.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "autonomy/common/network/backend/engine.hpp"
#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace perception
{

/** @brief Raw FrontierNet outputs for one frame. */
struct FrontierNetOutput
{
    /** @brief Distance field ``-log(d / normalizer)``, ``size * size``. */
    std::vector<float> df;
    /** @brief Gain-bin logits, ``classes * size * size`` (class-major). */
    std::vector<float> masks;
    /** @brief Number of gain bins. */
    std::uint32_t classes{0};
    /** @brief Square output side length in pixels. */
    std::uint32_t size{0};
};

/**
 * @brief ONNX FrontierNet graph: depth ``[1,3,S,S]`` -> ``df`` + ``masks``.
 *
 * @par Threading
 * Load once; Run from the navigation tick thread only.
 */
class FrontierNetModel final
{
public:
    /**
     * @brief Load the engine described by the manifest.
     *
     * @param[in] manifest Model manifest with one input and two outputs.
     * @param[out] error Filled on failure when non-null.
     */
    bool Load(const config::ModelManifest& manifest, std::string* error);

    /**
     * @brief Run one forward pass.
     *
     * @param[in] input ``3 * S * S`` DeFM-normalized CHW values.
     * @param[out] output Distance field and class logits.
     */
    bool Run(const std::vector<float>& input, FrontierNetOutput* output);

    /** @brief True after a successful Load. */
    [[nodiscard]] bool Ready() const;

    /** @brief Square input/output side length in pixels. */
    [[nodiscard]] std::uint32_t InputSize() const;

    /** @brief Number of gain bins the graph predicts. */
    [[nodiscard]] std::uint32_t Classes() const;

    /** @brief Backend name ("frontiernet_engine"). */
    [[nodiscard]] std::string_view Name() const;

private:
    /** @brief Owned inference engine. */
    std::unique_ptr<autonomy::common::network::Engine> engine_{nullptr};
    /** @brief Manifest input name for the depth tensor. */
    std::string depth_input_;
    /** @brief Manifest output name for the distance field. */
    std::string df_output_;
    /** @brief Manifest output name for the class logits. */
    std::string masks_output_;
    /** @brief Exact element counts from the validated static manifest. */
    std::size_t input_count_{0};
    std::size_t df_count_{0};
    std::size_t masks_count_{0};
    /** @brief Square side length in pixels. */
    std::uint32_t size_{0};
    /** @brief Number of gain bins. */
    std::uint32_t classes_{0};
};

/** @brief Factory used when a model manifest is available. */
std::unique_ptr<FrontierNetModel> CreateFrontierNetModel(
    const config::ModelManifest& manifest, std::string* error);

}  // namespace perception
}  // namespace navigation
