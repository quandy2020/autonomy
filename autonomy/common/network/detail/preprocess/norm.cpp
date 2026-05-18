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

#include "autonomy/common/network/detail/preprocess/norm.hpp"

namespace autonomy {
namespace common {
namespace network {
namespace internal {

namespace {

constexpr float kEpsilon = 1e-6f;

template <NormalizePolicy Policy>
void ApplyNormImpl(std::vector<float>* tensor, const NormalizeParams& custom, int channels,
                   int height, int width, bool swap_rb) {
    if constexpr (NormTraits<Policy>::kDivStd) {
        DivStd(tensor, channels, height, width, NormTraits<Policy>::StdRgb(custom), swap_rb);
    }
}

}  // namespace

void DivStd(std::vector<float>* tensor, int channels, int height, int width,
            const float std_rgb[3], bool swap_rb) {
    if (tensor == nullptr || tensor->empty()) {
        return;
    }
    const int plane = height * width;
    const float std_bgr[3] = {std_rgb[2], std_rgb[1], std_rgb[0]};
    const float* std_vals = swap_rb ? std_rgb : std_bgr;
    for (int c = 0; c < channels && c < 3; ++c) {
        if (std_vals[c] <= kEpsilon) {
            continue;
        }
        const size_t offset = static_cast<size_t>(c * plane);
        for (int i = 0; i < plane; ++i) {
            (*tensor)[offset + static_cast<size_t>(i)] /= std_vals[c];
        }
    }
}

void BlobParams(NormalizePolicy policy, const NormalizeParams& custom, double* scale,
                cv::Scalar* mean) {
    VisitNorm(policy, [&](auto tag) {
        constexpr NormalizePolicy selected = decltype(tag)::value;
        NormTraits<selected>::Blob(scale, mean, custom);
        return true;
    });
}

void ApplyNorm(std::vector<float>* tensor, NormalizePolicy policy, const NormalizeParams& custom,
               int channels, int height, int width, bool swap_rb) {
    VisitNorm(policy, [&](auto tag) {
        constexpr NormalizePolicy selected = decltype(tag)::value;
        ApplyNormImpl<selected>(tensor, custom, channels, height, width, swap_rb);
        return true;
    });
}

}  // namespace internal
}  // namespace network
}  // namespace common
}  // namespace autonomy
