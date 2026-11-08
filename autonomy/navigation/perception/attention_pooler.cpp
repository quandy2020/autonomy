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

#include "navigation/perception/attention_pooler.hpp"

#include <cmath>

namespace navigation
{
namespace perception
{

bool AttentionPooler::Pool(const std::vector<float>& tokens, std::uint32_t token_count,
                           std::uint32_t token_dim, std::vector<float>* pooled,
                           double* token_norm) const
{
    if (pooled == nullptr || token_count == 0 || token_dim == 0 ||
        tokens.size() < static_cast<std::size_t>(token_count) * token_dim)
    {
        return false;
    }
    pooled->assign(token_dim, 0.0F);
    for (std::uint32_t token = 0; token < token_count; ++token)
    {
        const std::size_t offset = static_cast<std::size_t>(token) * token_dim;
        for (std::uint32_t dim = 0; dim < token_dim; ++dim)
        {
            (*pooled)[dim] += tokens[offset + dim];
        }
    }
    double norm_squared = 0.0;
    for (float& value : *pooled)
    {
        value /= static_cast<float>(token_count);
        norm_squared += static_cast<double>(value) * value;
    }
    if (token_norm != nullptr)
    {
        *token_norm = std::sqrt(norm_squared);
    }
    return true;
}

}  // namespace perception
}  // namespace navigation
