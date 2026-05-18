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

#include "autonomy/common/network/detail/preprocess/resize.hpp"
#include "autonomy/common/network/detail/internal/resize_impl.hpp"

#include "autonomy/common/network/detail/internal/error.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <type_traits>
#include <utility>

namespace autonomy {
namespace common {
namespace network {

namespace internal {

namespace {

using internal::SetErrorMessage;

cv::Scalar PadBgr(int value) {
    return cv::Scalar(value, value, value);
}

bool LetterboxImpl(const cv::Mat& bgr, int h, int w, int pad_value,
                   cv::Mat* out, TransformMeta* meta,
                   LetterboxOutput* letterbox) {
    const double scale = std::min(h / static_cast<double>(bgr.rows),
                                  w / static_cast<double>(bgr.cols));
    const int sw = static_cast<int>(std::round(bgr.cols * scale));
    const int sh = static_cast<int>(std::round(bgr.rows * scale));
    const int pad_left = static_cast<int>(std::round((w - sw) / 2.0));
    const int pad_top = static_cast<int>(std::round((h - sh) / 2.0));

    *out = cv::Mat(h, w, bgr.type(), PadBgr(pad_value));
    cv::Mat scaled;
    cv::resize(bgr, scaled, cv::Size(sw, sh));
    scaled.copyTo((*out)(cv::Rect(pad_left, pad_top, sw, sh)));

    SetMeta(meta, ResizePolicy::kLetterbox, scale, pad_left, pad_top, 0, 0, h,
            w, bgr.rows, bgr.cols);

    if (letterbox != nullptr) {
        letterbox->image = *out;
        letterbox->scale_ratio = scale;
        letterbox->pad_width = w - sw;
        letterbox->pad_height = h - sh;
        if (meta != nullptr) {
            letterbox->meta = *meta;
        }
    }
    return true;
}

}  // namespace

void SetMeta(TransformMeta* meta, ResizePolicy policy, double scale,
             int pad_left, int pad_top, int crop_x, int crop_y, int in_h,
             int in_w, int src_h, int src_w) {
    if (meta == nullptr) {
        return;
    }
    meta->resize_policy = policy;
    meta->scale_gain = scale;
    meta->padding_left = pad_left;
    meta->padding_top = pad_top;
    meta->crop_offset_x = crop_x;
    meta->crop_offset_y = crop_y;
    meta->input_height = in_h;
    meta->input_width = in_w;
    meta->source_height = src_h;
    meta->source_width = src_w;
}

bool Letterbox(const cv::Mat& bgr, int h, int w, LetterboxOutput* out,
               int pad) {
    if (out == nullptr) {
        return false;
    }
    TransformMeta meta;
    return LetterboxImpl(bgr, h, w, pad, &out->image, &meta, out);
}

bool Stretch(const cv::Mat& bgr, int h, int w, cv::Mat* out,
             TransformMeta* meta) {
    cv::resize(bgr, *out, cv::Size(w, h));
    const double scale = std::min(h / static_cast<double>(bgr.rows),
                                  w / static_cast<double>(bgr.cols));
    SetMeta(meta, ResizePolicy::kStretch, scale, 0, 0, 0, 0, h, w, bgr.rows,
            bgr.cols);
    return true;
}

bool UpperBound(const cv::Mat& bgr, int bound, cv::Mat* out,
                TransformMeta* meta) {
    const int longest = std::max(bgr.rows, bgr.cols);
    const double scale = bound / static_cast<double>(longest);
    const int nw = static_cast<int>(std::round(bgr.cols * scale));
    const int nh = static_cast<int>(std::round(bgr.rows * scale));
    cv::resize(bgr, *out, cv::Size(nw, nh));
    SetMeta(meta, ResizePolicy::kUpperBound, scale, 0, 0, 0, 0, nh, nw,
            bgr.rows, bgr.cols);
    return true;
}

bool Align(cv::Mat* image, int multiple) {
    if (image == nullptr || multiple <= 0) {
        return false;
    }
    const int ah = static_cast<int>(
                       std::ceil(image->rows / static_cast<double>(multiple))) *
                   multiple;
    const int aw = static_cast<int>(
                       std::ceil(image->cols / static_cast<double>(multiple))) *
                   multiple;
    if (ah == image->rows && aw == image->cols) {
        return true;
    }
    cv::resize(*image, *image, cv::Size(aw, ah));
    return true;
}

bool CenterCrop(const cv::Mat& bgr, int h, int w, cv::Mat* out,
                TransformMeta* meta) {
    const double scale = std::max(h / static_cast<double>(bgr.rows),
                                  w / static_cast<double>(bgr.cols));
    const int sw = static_cast<int>(std::round(bgr.cols * scale));
    const int sh = static_cast<int>(std::round(bgr.rows * scale));
    cv::Mat scaled;
    cv::resize(bgr, scaled, cv::Size(sw, sh));
    const int crop_x = std::max(0, (sw - w) / 2);
    const int crop_y = std::max(0, (sh - h) / 2);
    *out = scaled(cv::Rect(crop_x, crop_y, w, h)).clone();
    SetMeta(meta, ResizePolicy::kCenterCrop, scale, 0, 0, crop_x, crop_y, h, w,
            bgr.rows, bgr.cols);
    return true;
}

template <ResizePolicy Policy>
bool ResizeFor(const cv::Mat& bgr, int h, int w, const PreprocessOptions& opt,
               cv::Mat* out, TransformMeta* meta, std::string* error) {
    if constexpr (Policy == ResizePolicy::kLetterbox) {
        LetterboxOutput letterbox;
        if (!Letterbox(bgr, h, w, &letterbox, opt.pad_value)) {
            SetErrorMessage(error, "letterbox failed.");
            return false;
        }
        *out = std::move(letterbox.image);
        if (meta != nullptr) {
            *meta = letterbox.meta;
        }
        return true;
    } else if constexpr (Policy == ResizePolicy::kStretch) {
        return Stretch(bgr, h, w, out, meta);
    } else if constexpr (Policy == ResizePolicy::kCenterCrop) {
        return CenterCrop(bgr, h, w, out, meta);
    } else if constexpr (Policy == ResizePolicy::kUpperBound) {
        const int bound = opt.bound_resize_target > 0 ? opt.bound_resize_target
                                                      : std::max(h, w);
        if (!UpperBound(bgr, bound, out, meta)) {
            SetErrorMessage(error, "upper bound resize failed.");
            return false;
        }
        if (opt.align_multiple > 0 && !Align(out, opt.align_multiple)) {
            SetErrorMessage(error, "align failed.");
            return false;
        }
        if (out->rows != h || out->cols != w) {
            cv::resize(*out, *out, cv::Size(w, h));
        }
        return true;
    } else {
        SetErrorMessage(error, "unknown resize policy.");
        return false;
    }
}

bool ResizeForPolicy(ResizePolicy policy, const cv::Mat& bgr, int h, int w,
                     const PreprocessOptions& opt, cv::Mat* out,
                     TransformMeta* meta, std::string* error) {
    return VisitResize(policy, [&](auto tag) {
        constexpr ResizePolicy selected = decltype(tag)::value;
        return ResizeFor<selected>(bgr, h, w, opt, out, meta, error);
    });
}

}  // namespace internal

bool Resize(const cv::Mat& bgr, int height, int width,
            const PreprocessOptions& opt, cv::Mat* out, TransformMeta* meta,
            std::string* error) {
    return internal::ResizeForPolicy(opt.resize, bgr, height, width, opt, out,
                                     meta, error);
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
