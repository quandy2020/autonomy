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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_INTERNAL_TRAITS_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_INTERNAL_TRAITS_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <opencv2/core.hpp>

#include <type_traits>
#include <utility>
#include <vector>

namespace autonomy {
namespace common {
namespace network {
namespace preprocess_internal {

/**
 * @file traits.hpp
 * @brief Compile-time policy traits and enum visitors
 */

template <NormalizePolicy Policy>
struct NormTraits;

template <>
struct NormTraits<NormalizePolicy::kZeroOne> {
    static constexpr bool kDivStd = false;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams&) {
        *scale = 1.0 / 255.0;
        *mean = cv::Scalar(0., 0., 0.);
    }
};

template <>
struct NormTraits<NormalizePolicy::kMinusOneToOne> {
    static constexpr bool kDivStd = false;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams&) {
        *scale = 2.0 / 255.0;
        *mean = cv::Scalar(127.5, 127.5, 127.5);
    }
};

template <>
struct NormTraits<NormalizePolicy::kImageNet> {
    static constexpr bool kDivStd = true;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams&) {
        constexpr float kMeanRgb[3] = {0.485f, 0.456f, 0.406f};
        *scale = 1.0 / 255.0;
        *mean = cv::Scalar(kMeanRgb[0] * 255.f, kMeanRgb[1] * 255.f, kMeanRgb[2] * 255.f);
    }

    static const float* StdRgb(const NormalizeParams&) {
        static constexpr float kStdRgb[3] = {0.229f, 0.224f, 0.225f};
        return kStdRgb;
    }
};

template <>
struct NormTraits<NormalizePolicy::kCustom> {
    static constexpr bool kDivStd = true;

    static void Blob(double* scale, cv::Scalar* mean, const NormalizeParams& custom) {
        *scale = 1.0 / 255.0;
        *mean = cv::Scalar(custom.mean[0] * 255.f, custom.mean[1] * 255.f,
                           custom.mean[2] * 255.f);
    }

    static const float* StdRgb(const NormalizeParams& custom) { return custom.std; }
};

template <LayoutPolicy Layout>
struct LayoutTraits {
    static constexpr bool kToNhwc = (Layout == LayoutPolicy::kNHWC);
};

template <ResizePolicy Policy>
struct ResizeTraits {
    static constexpr ResizePolicy kPolicy = Policy;
};

template <typename Callable>
bool VisitNorm(NormalizePolicy policy, Callable&& fn) {
    switch (policy) {
        case NormalizePolicy::kZeroOne:
            return fn(std::integral_constant<NormalizePolicy, NormalizePolicy::kZeroOne>{});
        case NormalizePolicy::kMinusOneToOne:
            return fn(
                std::integral_constant<NormalizePolicy, NormalizePolicy::kMinusOneToOne>{});
        case NormalizePolicy::kImageNet:
            return fn(std::integral_constant<NormalizePolicy, NormalizePolicy::kImageNet>{});
        case NormalizePolicy::kCustom:
            return fn(std::integral_constant<NormalizePolicy, NormalizePolicy::kCustom>{});
    }
    return false;
}

template <typename Callable>
bool VisitLayout(LayoutPolicy policy, Callable&& fn) {
    switch (policy) {
        case LayoutPolicy::kNCHW:
            return fn(std::integral_constant<LayoutPolicy, LayoutPolicy::kNCHW>{});
        case LayoutPolicy::kNHWC:
            return fn(std::integral_constant<LayoutPolicy, LayoutPolicy::kNHWC>{});
        case LayoutPolicy::kAuto:
            break;
    }
    return false;
}

template <typename Callable>
bool VisitResize(ResizePolicy policy, Callable&& fn) {
    switch (policy) {
        case ResizePolicy::kLetterbox:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kLetterbox>{});
        case ResizePolicy::kStretch:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kStretch>{});
        case ResizePolicy::kCenterCrop:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kCenterCrop>{});
        case ResizePolicy::kUpperBound:
            return fn(std::integral_constant<ResizePolicy, ResizePolicy::kUpperBound>{});
    }
    return false;
}

}  // namespace preprocess_internal
}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_INTERNAL_TRAITS_HPP_
