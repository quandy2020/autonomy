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

#include "autonomy/common/network/detail/postprocess/cls.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"

#include <algorithm>

namespace autonomy {
namespace common {
namespace network {

namespace {
using internal::SetErrorMessage;
}  // namespace

bool TopK(const std::vector<float>& logits, int top_k, std::vector<ClassScore>* result,
          std::string* error) {
    if (result == nullptr) {
        SetErrorMessage(error, "result output is null.");
        return false;
    }
    if (logits.empty()) {
        SetErrorMessage(error, "empty logits.");
        return false;
    }
    const int k = std::max(1, std::min(top_k, static_cast<int>(logits.size())));
    std::vector<int> indices(logits.size());
    for (size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<int>(index);
    }
    std::partial_sort(indices.begin(), indices.begin() + k, indices.end(),
                      [&logits](int left, int right) { return logits[left] > logits[right]; });
    result->clear();
    result->reserve(static_cast<size_t>(k));
    for (int rank = 0; rank < k; ++rank) {
        const int class_id = indices[static_cast<size_t>(rank)];
        result->push_back({class_id, logits[static_cast<size_t>(class_id)]});
    }
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
