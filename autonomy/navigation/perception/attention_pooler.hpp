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
 * @file attention_pooler.hpp
 * @brief Token pooling for the frozen DeFM encoder.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace navigation
{
namespace perception
{

class AttentionPooler
{
public:
    /**
     * @brief Mean-pool tokens into one embedding.
     *
     * @param[in] tokens Row-major [token_count, token_dim] buffer.
     * @param[in] token_count Number of tokens.
     * @param[in] token_dim Token dimension.
     * @param[out] pooled Mean embedding of length token_dim.
     * @param[out] token_norm Optional L2 norm of the pooled embedding.
     * @return false on malformed shapes.
     */
    bool Pool(const std::vector<float>& tokens, std::uint32_t token_count,
              std::uint32_t token_dim, std::vector<float>* pooled,
              double* token_norm) const;
};

}  // namespace perception
}  // namespace navigation
