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

#include "autonomy/common/network/detail/preprocess/layout.hpp"

#include "autonomy/common/network/detail/preprocess/dims.hpp"

namespace autonomy {
namespace common {
namespace network {

LayoutPolicy ResolveLayout(LayoutPolicy requested, const std::vector<int64_t>& dims) {
    if (requested != LayoutPolicy::kAuto) {
        return requested;
    }
    if (dims.size() == 4 && preprocess_internal::IsChannelDim(dims[3])) {
        return LayoutPolicy::kNHWC;
    }
    return LayoutPolicy::kNCHW;
}

bool ToNhwc(const std::vector<float>& nchw, int channels, int height, int width,
            std::vector<float>* nhwc) {
    const size_t plane = static_cast<size_t>(height * width);
    if (nchw.size() < plane * static_cast<size_t>(channels)) {
        return false;
    }
    nhwc->resize(nchw.size());
    for (int c = 0; c < channels; ++c) {
        for (int r = 0; r < height; ++r) {
            for (int col = 0; col < width; ++col) {
                const size_t nchw_i = static_cast<size_t>(c * plane + r * width + col);
                const size_t nhwc_i =
                    static_cast<size_t>((r * width + col) * channels + c);
                (*nhwc)[nhwc_i] = nchw[nchw_i];
            }
        }
    }
    return true;
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
