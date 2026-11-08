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
 * @file defm_model.hpp
 * @brief Backend boundary for the frozen DeFM model.
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace perception
{

/** @brief DeFM forward-pass output. */
struct DefmOutput
{
    /** @brief Row-major [token_count, token_dim] tokens. */
    std::vector<float> tokens;
    /** @brief Pooled embedding of length token_dim. */
    std::vector<float> pooled;
    /** @brief Number of tokens produced. */
    std::uint32_t token_count{0};
    /** @brief Token dimension. */
    std::uint32_t token_dim{0};
};

/**
 * @brief DeFM inference backend.
 *
 * @par Threading
 * Load runs once; Run is called from the navigation tick thread.
 */
class DefmModel
{
public:
    virtual ~DefmModel();

    /**
     * @brief Load weights and validate tensor shapes.
     *
     * @param[in] manifest Model path, sha256 and I/O specification.
     * @param[out] error Human-readable failure reason.
     * @return true when the backend is ready.
     */
    virtual bool Load(const config::ModelManifest& manifest, std::string* error) = 0;

    /**
     * @brief Run one forward pass.
     *
     * @param[in] input Normalized CHW depth tensor.
     * @param[out] output Tokens/pooled embedding.
     * @return false on inference failure or shape mismatch.
     */
    virtual bool Run(const std::vector<float>& input, DefmOutput* output) = 0;

    /** @brief Stable backend name for diagnostics. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

}  // namespace perception
}  // namespace navigation
