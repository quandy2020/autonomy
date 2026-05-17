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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_LAYOUT_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_LAYOUT_HPP_

#include "autonomy/common/network/detail/preprocess/internal/traits.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <cstdint>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file layout.hpp
 * @brief NCHW / NHWC layout resolution and conversion
 */

LayoutPolicy ResolveLayout(LayoutPolicy requested, const std::vector<int64_t>& dims);

bool ToNhwc(const std::vector<float>& nchw, int channels, int height, int width,
            std::vector<float>* nhwc);

namespace preprocess_internal {

template <LayoutPolicy Layout>
bool ToLayout(std::vector<float> nchw, int channels, int height, int width,
              std::vector<float>* out) {
    if constexpr (LayoutTraits<Layout>::kToNhwc) {
        return ToNhwc(nchw, channels, height, width, out);
    } else {
        *out = std::move(nchw);
        return true;
    }
}

}  // namespace preprocess_internal

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_LAYOUT_HPP_
