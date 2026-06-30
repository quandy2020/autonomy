/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_UTIL_FANCY_INDEX_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_UTIL_FANCY_INDEX_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <vector>
#include <type_traits>

namespace autonomy::localization::atlas {
namespace util {

template<typename T, typename U>
std::vector<T> resample_by_indices(const std::vector<T>& elements, const std::vector<U>& indices) {
    static_assert(std::is_integral<U>(), "the element type of indices must be integer");

    std::vector<T> resampled;
    resampled.reserve(elements.size());
    for (const auto idx : indices) {
        resampled.push_back(elements.at(idx));
    }

    return resampled;
}

template<typename T, typename U>
eigen_alloc_vector<T> resample_by_indices(const eigen_alloc_vector<T>& elements, const std::vector<U>& indices) {
    static_assert(std::is_integral<U>(), "the element type of indices must be integer");

    eigen_alloc_vector<T> resampled;
    resampled.reserve(elements.size());
    for (const auto idx : indices) {
        resampled.push_back(elements.at(idx));
    }

    return resampled;
}

template<typename T>
std::vector<T> resample_by_indices(const std::vector<T>& elements, const std::vector<bool>& indices) {
    assert(elements.size() == indices.size());

    std::vector<T> resampled;
    resampled.reserve(elements.size());
    for (unsigned int idx = 0; idx < elements.size(); ++idx) {
        if (indices.at(idx)) {
            resampled.push_back(elements.at(idx));
        }
    }

    return resampled;
}

template<typename T>
eigen_alloc_vector<T> resample_by_indices(const eigen_alloc_vector<T>& elements, const std::vector<bool>& indices) {
    assert(elements.size() == indices.size());

    eigen_alloc_vector<T> resampled;
    resampled.reserve(elements.size());
    for (unsigned int idx = 0; idx < elements.size(); ++idx) {
        if (indices.at(idx)) {
            resampled.push_back(elements.at(idx));
        }
    }

    return resampled;
}

} // namespace util
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_UTIL_FANCY_INDEX_HPP_
